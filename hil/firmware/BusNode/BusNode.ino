// Board A: the actuator-bus node. It ACKs and receives what board B puts on the real CAN bus and forwards every
// frame to the PC ('M': id, length, data), so the bench sees B's outputs only through the bus.
// Optional: drives the hardware watchdog line to board B ('K' from the PC -> one pulse per kick on GPIO5).
// Diagnostics every 200 ms ('D'): frames received, MCP2515 error flags, receive overflows.
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <ssc_board.h>
#include <ssc_core.h>

static MCP2515 mcp(board::PIN_CAN_CS);
static bool can_ok = false;
static ssc::Parser parser;
static uint32_t rx_count = 0, overflows = 0, last_diag = 0, last_hello = 0;
static char fw[64];

static void send(uint8_t type, const uint8_t *p, uint8_t n) {
  uint8_t buf[260];
  size_t k = ssc::frame_msg(type, p, n, buf);
  Serial.write(buf, k);
}

void setup() {
  Serial.setRxBufferSize(1024);
  Serial.begin(board::LINK_BAUD);
  pinMode(board::PIN_WD_LINE, OUTPUT);
  digitalWrite(board::PIN_WD_LINE, LOW);
  const char *xtal = "none";
  can_ok = board::mcp_init(mcp, &xtal);
  snprintf(fw, sizeof fw, "BusNode 2.3 (A) CAN %s", can_ok ? xtal : "FAILED");
}

void loop() {
  if (can_ok) {
    struct can_frame f;
    while (mcp.readMessage(&f) == MCP2515::ERROR_OK) {
      uint8_t m[11] = {(uint8_t)(f.can_id & 0xFF), (uint8_t)((f.can_id >> 8) & 0x07), f.can_dlc};
      memcpy(m + 3, f.data, f.can_dlc > 8 ? 8 : f.can_dlc);
      send('M', m, (uint8_t)(3 + (f.can_dlc > 8 ? 8 : f.can_dlc)));
      rx_count++;
    }
  }
  int n;
  while ((n = Serial.available()) > 0) {
    uint8_t b = (uint8_t)Serial.read();
    if (parser.push(b) && parser.type == 'K' && parser.len >= 1) {
      for (int i = 0; i < parser.payload[0]; i++) {
        digitalWrite(board::PIN_WD_LINE, HIGH);
        delayMicroseconds(50);
        digitalWrite(board::PIN_WD_LINE, LOW);
        delayMicroseconds(50);
      }
    }
  }
  uint32_t now = millis();
  if (now - last_hello >= 1000) {
    last_hello = now;
    send('H', (const uint8_t *)fw, (uint8_t)strlen(fw));
  }
  if (can_ok && now - last_diag >= 200) {
    last_diag = now;
    uint8_t e = mcp.getErrorFlags();
    if (e & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR)) {
      overflows++;
      mcp.clearRXnOVR();
    }
    uint8_t d[9] = {(uint8_t)rx_count, (uint8_t)(rx_count >> 8), (uint8_t)(rx_count >> 16), (uint8_t)(rx_count >> 24), e,
                    (uint8_t)overflows, (uint8_t)(overflows >> 8), (uint8_t)(overflows >> 16), (uint8_t)(overflows >> 24)};
    send('D', d, sizeof d);
  }
}
