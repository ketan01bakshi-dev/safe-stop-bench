// Board A: the actuator-bus node. It ACKs and receives what board B puts on the real CAN bus and forwards every
// frame to the PC ('M': id, length, data), so the bench sees B's outputs only through the bus.
// Optional: drives the hardware watchdog line to board B ('K' from the PC -> one pulse per kick on GPIO5).
// Diagnostics every 200 ms ('D'): frames forwarded, MCP2515 error flags, receive overflows, last bad operating mode,
// controller re-inits, single-read SPI glitches, RXB1 flag misreads, RXB0CTRL, and (2.6) the RAM queue's high-water
// mark and drops. Every 5 ms the MCP2515 is checked to be in normal mode and re-initialised if not (v2.9.3); a re-init
// sends 'D' at once, so the bench can discard the scenario as a bench fault.
//
// v2.9.8 (BusNode 2.6): CAN reception runs in its own FreeRTOS task on core 0, woken by the MCP2515's INT line (and
// every 1 ms as a fallback), and puts frames into a RAM queue; loop() on core 1 only drains the queue to USB and
// drives the watchdog pulses. Found on hardware: a burst of 30 kicks is 3 ms of busy-wait pulses, and with rollover off
// (v2.9.4) RXB0 is A's only receive buffer, so B's back-to-back frames overflowed it: 40 overflows in 60 s with bursts,
// 0 without. All SPI traffic now happens in the CAN task, so the two cores never share the bus.
//
// v2.12 (BusNode 2.7): power cut of board B. 'X' <ms:u16> from the PC energises the relay on GPIO6 for that long
// (non-blocking, at most 60 s); the relay's normally-closed contact is in B's 5 V supply, so B loses power and comes back
// when it releases. The pin is low at boot, so an A that resets or hangs in boot never leaves B unpowered.
//
// v2.23 (BusNode 2.8): bench-health UDS. 'T' <id:u16> <dlc> <data> from the PC puts one standard CAN frame on the bus (the
// bench's tester port: UDS requests on 0x7DF / 0x7E2 / 0x7E3). A answers UDS itself on 0x7E3 -> 0x7EB (and 0x7DF) with
// TesterPresent and ReadDataByIdentifier F195 (version) / F18C (serial); its answer is also forwarded to the PC ('M'),
// because a node does not receive its own frames. Transmit happens in the CAN task like all other SPI traffic.
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <ssc_board.h>
#include <ssc_core.h>
#include <ssc_uds.h>

#define FW_VERSION "2.12"   // v2.29: receive-latency diagnostics (lat_* below); the CAN interrupt is now installed on the CAN task's core

#ifndef CAN_SPI_HZ
#define CAN_SPI_HZ 10000000   // the library default
#endif
#ifndef SSC_MODE_CHECK
#define SSC_MODE_CHECK 1
#endif
static const int QUEUE_LEN = 256;   // 1.28 s of B's traffic (200 frames/s)

static MCP2515 mcp(board::PIN_CAN_CS, CAN_SPI_HZ);
static ssc::Parser parser;
static char fw[64];
static uint32_t last_diag = 0, last_hello = 0;
static uint32_t relay_on_at = 0, relay_ms = 0;   // power cut of board B in progress (relay_ms > 0)

// shared between the CAN task (writer) and loop() (reader); 32-bit and smaller stores are atomic on the ESP32-S3
static volatile bool can_ok = false, can_started = false, force_diag = false;
static volatile uint32_t rx_count = 0, overflows = 0, q_drops = 0, int_cleared = 0, rx1_cleared = 0;
static volatile uint16_t reinits = 0, glitches = 0, rx1_mismatch = 0, q_high = 0;
static volatile uint8_t last_eflg = 0, bad_mode = 0, last_stat = 0, bukt = 0xEE;
// v2.29: how long the CAN task takes to react to a frame, to find out why the single receive buffer sometimes overflows.
// lat = from the INT edge (stamped in the ISR) to the first line of the task after it wakes; drain = time spent in drain_rxb0().
// B sends two frames about 230 us apart and RXB0 holds one, so a latency + drain above ~230 us loses the second frame.
static volatile uint32_t int_us = 0, last_lat = 0, last_drain = 0, lat_max = 0, drain_max = 0, ovf_lat = 0, ovf_drain = 0;
static volatile uint32_t lat_hist[5] = {0, 0, 0, 0, 0};   // < 50, < 100, < 230, < 1000, >= 1000 us
// The overflow flag is read every 2 ms, so "what the task was doing" has to be the worst of the whole 2 ms window, not the last wake.
// win_* are reset at every flag read; each overflow is filed under every cause that was present in its window (they can overlap),
// and under ovf_none when nothing was.  [0] slow wake >= 230 us  [1] long drain >= 150 us  [2] RXB1 misread  [3] SPI mode glitch  [4] none
static volatile uint32_t win_lat = 0, win_drain = 0, win_mismatch = 0, win_glitch = 0, ovf_by[5] = {0, 0, 0, 0, 0};
// The number that decides it: time from the CAN interrupt until RXB0 is FREE again (RX0IF cleared). B's frames are back to back
// (~222 us each at 500 kbit/s), so the next frame overflows the single buffer if this exceeds roughly 220 us.
static volatile uint32_t wake_us = 0, win_clr = 0, clr_max = 0, clr_hist[5] = {0, 0, 0, 0, 0};   // < 50, < 100, < 150, < 200, >= 200 us
static QueueHandle_t rxq, txq;
static ssc::uds::Node uds_node = {ssc::uds::ID_A_REQ, ssc::uds::ID_A_RESP, FW_VERSION, 0};
static TaskHandle_t can_task_h;

static void send(uint8_t type, const uint8_t *p, uint8_t n) {
  uint8_t buf[260];
  size_t k = ssc::frame_msg(type, p, n, buf);
  Serial.write(buf, k);
}

static void IRAM_ATTR on_can_int() {
  int_us = micros();
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(can_task_h, &woken);
  portYIELD_FROM_ISR(woken);
}

// v2.29: read RXB0 in ONE SPI transaction. READ RX BUFFER (0x90) returns SIDH..D7 and the chip clears RX0IF itself when CS goes
// high, so the buffer is free ~25 us after the task starts instead of after the library's separate header read, data read and flag
// clear. RTR is not decoded (the bench sends none); IDE/extended ids are, as the library does.
static bool read_rxb0_fast(struct can_frame *f) {
  uint8_t tx[14] = {0x90}, rx[14];
  SPI.beginTransaction(SPISettings(CAN_SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(board::PIN_CAN_CS, LOW);
  SPI.transferBytes(tx, rx, sizeof tx);
  digitalWrite(board::PIN_CAN_CS, HIGH);
  SPI.endTransaction();
  uint8_t dlc = rx[5] & 0x0F;
  if (dlc > 8) return false;
  uint32_t id = ((uint32_t)rx[1] << 3) + (rx[2] >> 5);
  if (rx[2] & 0x08) {   // extended frame
    id = (id << 2) + (rx[2] & 0x03);
    id = (id << 8) + rx[3];
    id = (id << 8) + rx[4];
    id |= CAN_EFF_FLAG;
  }
  f->can_id = id;
  f->can_dlc = dlc;
  memcpy(f->data, rx + 6, dlc);
  return true;
}

// v2.9.4: receive from RXB0 only. At the instant a frame lands, the MCP2515 sometimes returns its flag register shifted
// by one bit (0x02 "RXB1 full" for 0x01 "RXB0 full"); with rollover off and accept-all filters RXB1 never holds a new
// frame, so an RXB1 flag is a misread: count it and read the flags again.
static void drain_rxb0() {
  struct can_frame f;
  for (int tries = 0; tries < 8; tries++) {
    uint8_t intf = board::mcp_read_reg(0x2C, CAN_SPI_HZ);
    if (intf & 0xA0) {
      // ERRIF / MERRF: INT is level-active, so while either is set INT stays low and no new RX edge can wake us.
      // ERRIF is set on ANY error-flag change, including back to "no errors", so clear it whenever it shows.
      board::mcp_bitmod(0x2C, 0xA0, 0x00, CAN_SPI_HZ);
      int_cleared++;
    }
    if (!(intf & 0x01)) {
      if (intf & 0x02) {
        rx1_mismatch++;
        win_mismatch++;
        last_stat = intf;
        if (board::mcp_read_reg(0x2C, CAN_SPI_HZ) & 0x02) {
          board::mcp_bitmod(0x2C, 0x02, 0x00, CAN_SPI_HZ);   // a real, persisting RX1IF: RXB1 is stale, drop it
          rx1_cleared++;
        }
        continue;
      }
      return;
    }
    if (!read_rxb0_fast(&f)) return;
    if (wake_us) {   // only the first frame of a wake has a meaningful interrupt stamp
      uint32_t tc = micros() - wake_us;
      wake_us = 0;
      if (tc > win_clr) win_clr = tc;
      if (tc > clr_max) clr_max = tc;
      clr_hist[tc < 50 ? 0 : tc < 100 ? 1 : tc < 150 ? 2 : tc < 200 ? 3 : 4]++;
    }
    if (xQueueSend(rxq, &f, 0) != pdTRUE) q_drops++;
    uds_answer(f);
    UBaseType_t used = QUEUE_LEN - uxQueueSpacesAvailable(rxq);
    if (used > q_high) q_high = (uint16_t)used;
  }
}

// A UDS request addressed to A (seen on the bus, or sent by A itself for the PC): answer on the bus and tell the PC.
static void uds_answer(const struct can_frame &f) {
  if (f.can_id & CAN_EFF_FLAG) return;
  uint8_t out[8];
  uint8_t k = ssc::uds::respond(uds_node, (uint16_t)(f.can_id & 0x7FF), f.data, f.can_dlc, out);
  if (!k) return;
  struct can_frame r;
  r.can_id = uds_node.resp_id;
  r.can_dlc = k;
  memcpy(r.data, out, k);
  mcp.sendMessage(&r);
  if (xQueueSend(rxq, &r, 0) != pdTRUE) q_drops++;
}

static void can_task(void *) {
  // v2.29: install the GPIO interrupt FROM THIS TASK (core 0). On the ESP32 a GPIO interrupt is serviced on the core that installed it;
  // from setup() that is core 1, which is busy with serial output and the kick pulses, so every frame's interrupt ran there and then had
  // to wake a task on the other core. Measured before: 1 wake in ~500 000 took 441 us, longer than the ~230 us between B's two
  // back-to-back frames, and the second frame found the single receive buffer still full.
  can_task_h = xTaskGetCurrentTaskHandle();
  pinMode(board::PIN_CAN_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(board::PIN_CAN_INT), on_can_int, FALLING);
  uint32_t last_flags = 0, last_mode = 0;
  bool counting = false;
  for (;;) {
    // INT, or 1 ms at the latest; and if INT is still low after a pass (a flag we did not clear), go round again at
    // once instead of waiting for an edge that cannot come
    // (at most 4 passes in a row, so a line held low by something else can't starve core 0's idle task)
    static int low_passes = 0;
    uint32_t notified = 0;
    if (digitalRead(board::PIN_CAN_INT) == HIGH || ++low_passes > 4) {
      low_passes = 0;
      notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
    }
    uint32_t t0 = micros();
    if (notified) {
      uint32_t lat = t0 - int_us;
      last_lat = lat;
      if (lat > win_lat) win_lat = lat;
      if (lat > lat_max) lat_max = lat;
      lat_hist[lat < 50 ? 0 : lat < 100 ? 1 : lat < 230 ? 2 : lat < 1000 ? 3 : 4]++;
    }
    wake_us = notified ? int_us : t0;
    uint32_t now = millis();
    if (can_ok) drain_rxb0();
    uint32_t dd = micros() - t0;
    last_drain = dd;
    if (dd > win_drain) win_drain = dd;
    if (dd > drain_max) drain_max = dd;
    struct can_frame tx;
    while (can_ok && xQueueReceive(txq, &tx, 0) == pdTRUE) {   // frames the PC asked A to send ('T')
      mcp.sendMessage(&tx);
      uds_answer(tx);   // a request for A itself: A does not receive its own frame
    }
    if (can_ok && now - last_flags >= 2) {
      last_flags = now;
      uint8_t e = mcp.getErrorFlags();
      last_eflg = e;
      if (e & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR)) {
        // boot window (B already transmitting while A starts, before this task ran) is not counted: no scenario runs then
        if (counting) {
          overflows++;
          ovf_lat = win_lat;        // the worst wake / drain of the 2 ms window in which the overflow showed
          ovf_drain = win_drain;
          bool any = false;
          if (win_lat >= 230) { ovf_by[0]++; any = true; }
          if (win_clr >= 150) { ovf_by[1]++; any = true; }   // buffer was held >= 150 us after the interrupt
          if (win_mismatch) { ovf_by[2]++; any = true; }
          if (win_glitch) { ovf_by[3]++; any = true; }
          if (!any) ovf_by[4]++;
        }
        mcp.clearRXnOVR();
      }
      counting = true;
      win_lat = win_drain = win_mismatch = win_glitch = win_clr = 0;   // next window
    }
    if (SSC_MODE_CHECK && can_started && now - last_mode >= 5) {
      last_mode = now;
      bool glitch;
      uint8_t om = board::mcp_opmode_confirmed(&glitch);
      if (glitch) {
        glitches++;
        win_glitch++;
      }
      if (om != 0) {
        bad_mode = om;
        reinits++;
        const char *x = "none";
        can_ok = board::mcp_init(mcp, &x);
        force_diag = true;
      }
    }
  }
}

void setup() {
  Serial.setRxBufferSize(1024);
  Serial.setTxBufferSize(4096);
  Serial.begin(board::LINK_BAUD);
  pinMode(board::PIN_WD_LINE, OUTPUT);
  digitalWrite(board::PIN_WD_LINE, LOW);
  pinMode(board::PIN_RELAY, OUTPUT);
  digitalWrite(board::PIN_RELAY, LOW);   // relay off: B powered
  const char *xtal = "none";
  can_ok = board::mcp_init(mcp, &xtal);
  can_started = can_ok;
  bukt = can_ok ? board::mcp_read_reg(0x60) : 0xEE;   // diagnostics: RXB0CTRL (BUKT must be 0)
  uds_node.serial = (uint32_t)ESP.getEfuseMac();
  snprintf(fw, sizeof fw, "BusNode " FW_VERSION " (A) CAN %s", can_ok ? xtal : "FAILED");
  rxq = xQueueCreate(QUEUE_LEN, sizeof(struct can_frame));
  txq = xQueueCreate(8, sizeof(struct can_frame));
  xTaskCreatePinnedToCore(can_task, "can_rx", 4096, nullptr, configMAX_PRIORITIES - 2, &can_task_h, 0);
}

void loop() {
  struct can_frame f;
  while (xQueueReceive(rxq, &f, 0) == pdTRUE) {
    uint8_t m[11] = {(uint8_t)(f.can_id & 0xFF), (uint8_t)((f.can_id >> 8) & 0x07), f.can_dlc};
    memcpy(m + 3, f.data, f.can_dlc > 8 ? 8 : f.can_dlc);
    send('M', m, (uint8_t)(3 + (f.can_dlc > 8 ? 8 : f.can_dlc)));
    rx_count++;
  }
  int n;
  while ((n = Serial.available()) > 0) {
    uint8_t b = (uint8_t)Serial.read();
    if (!parser.push(b)) continue;
    if (parser.type == 'K' && parser.len >= 1) {
      for (int i = 0; i < parser.payload[0]; i++) {   // busy-wait pulses: harmless now, CAN runs on the other core
        digitalWrite(board::PIN_WD_LINE, HIGH);
        delayMicroseconds(50);
        digitalWrite(board::PIN_WD_LINE, LOW);
        delayMicroseconds(50);
      }
    } else if (parser.type == 'T' && parser.len >= 3 && parser.payload[2] <= 8 && parser.len >= 3 + parser.payload[2]) {
      struct can_frame t;
      t.can_id = (uint32_t)parser.payload[0] | (uint32_t)(parser.payload[1] & 0x07) << 8;
      t.can_dlc = parser.payload[2];
      memcpy(t.data, parser.payload + 3, t.can_dlc);
      xQueueSend(txq, &t, 0);
    } else if (parser.type == 'X' && parser.len >= 2 && relay_ms == 0) {
      uint32_t ms = (uint32_t)parser.payload[0] | (uint32_t)parser.payload[1] << 8;
      relay_ms = ms > 60000 ? 60000 : (ms ? ms : 1);
      relay_on_at = millis();
      digitalWrite(board::PIN_RELAY, HIGH);   // relay on: B's supply open
    }
  }
  if (relay_ms && millis() - relay_on_at >= relay_ms) {
    digitalWrite(board::PIN_RELAY, LOW);      // relay off: B powered again
    relay_ms = 0;
  }
  uint32_t now = millis();
  if (now - last_hello >= 1000) {
    last_hello = now;
    send('H', (const uint8_t *)fw, (uint8_t)strlen(fw));
  }
  bool forced = force_diag;
  if ((can_ok || forced) && (forced || now - last_diag >= 200)) {
    force_diag = false;
    last_diag = now;
    uint32_t rx = rx_count, ov = overflows, dr = q_drops;
    uint16_t ri = reinits, gl = glitches, mm = rx1_mismatch, qh = q_high;
    uint32_t lh[5], lm = lat_max, dm = drain_max, ol = ovf_lat, od = ovf_drain;
    for (int i = 0; i < 5; i++) lh[i] = lat_hist[i];
    uint32_t ob[5];
    for (int i = 0; i < 5; i++) ob[i] = ovf_by[i];
    uint32_t ch[5], cm = clr_max;
    for (int i = 0; i < 5; i++) ch[i] = clr_hist[i];
    uint8_t d[111] = {(uint8_t)rx, (uint8_t)(rx >> 8), (uint8_t)(rx >> 16), (uint8_t)(rx >> 24), last_eflg,
                     (uint8_t)ov, (uint8_t)(ov >> 8), (uint8_t)(ov >> 16), (uint8_t)(ov >> 24),
                     bad_mode, (uint8_t)ri, (uint8_t)(ri >> 8), (uint8_t)gl, (uint8_t)(gl >> 8),
                     0, 0, (uint8_t)mm, (uint8_t)(mm >> 8), last_stat, 0, 0, 0, 0, 0, bukt,
                     (uint8_t)qh, (uint8_t)(qh >> 8), (uint8_t)dr, (uint8_t)(dr >> 8), (uint8_t)(dr >> 16), (uint8_t)(dr >> 24)};
    // v2.29: after byte 30, little-endian u32 each: five latency buckets, max latency, max drain, latency and drain at the last overflow
    uint32_t extra[9] = {lh[0], lh[1], lh[2], lh[3], lh[4], lm, dm, ol, od};
    for (int i = 0; i < 9; i++) memcpy(d + 31 + 4 * i, &extra[i], 4);
    for (int i = 0; i < 5; i++) memcpy(d + 67 + 4 * i, &ob[i], 4);   // overflows by coinciding cause (see ovf_by)
    for (int i = 0; i < 5; i++) memcpy(d + 87 + 4 * i, &ch[i], 4);   // interrupt -> buffer free, five buckets, then the maximum
    memcpy(d + 107, &cm, 4);
    send('D', d, sizeof d);
  }
}
