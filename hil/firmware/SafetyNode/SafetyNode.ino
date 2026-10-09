// Board B: the safety controller under test (HiL device under test).
//
// All logic is in SafeStopCore (ssc::Node + ssc::Controller), the same C++ that the bench runs on the PC and proves
// back-to-back against the Python reference. This file is only the shim: USB serial, MCP2515 CAN, millis/micros and
// the optional hardware watchdog line.
//
// Outputs every 10 ms: SAF_ActuatorCmd 0x200 (E2E Profile 2) and SAF_Status 0x201 on the real CAN bus, plus a copy of
// each on USB ('M') and diagnostics ('D': cycle count, worst execution time in us, worst lateness in ms).
// A CAN transmit error-passive or bus-off flag from the MCP2515 counts as an actuator-bus fault (ACT_BUS_OFF), if the
// bench enabled bus monitoring at reset (it does for two boards; one board alone gets no ACK, so it would always fault).
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <ssc_board.h>
#include <ssc_core.h>
#include <ssc_uds.h>
#include <Preferences.h>
#include "ssc_ota.h"

// v2.26: the minor version is a build flag (-DFW_MINOR=10), so the bench can build several signed images from one source.
// -DOTA_UNHEALTHY=1 builds an image whose health check fails: the OTA rollback test.
#ifndef FW_MINOR
#define FW_MINOR 10   // 2.10 = the baseline flashed by cable that can be updated over OTA (v2.26). 2.8 = steering rate-limit fix, 2.7 = UDS
#endif
#define STR_(x) #x
#define STR(x) STR_(x)
#define FW_VERSION "2." STR(FW_MINOR)

static MCP2515 mcp(board::PIN_CAN_CS);
static bool can_ok = false;
static ssc::Node node;
static volatile uint32_t wd_edges = 0;
static uint32_t last_flags_ms = 0;
static char fw[64];
static uint8_t stored[256], stored_n = 0;
static uint32_t saved_seq = 0;
static bool restored = false;
static bool pc_session = false;   // a boot-time restore leaves the node 'active' in its latched stop; that is not a run in progress
static ssc::Parser ota_parser;   // a second parser on the same bytes: it only reacts to the 'U' (OTA) messages
static ssc::uds::Node uds_node = {ssc::uds::ID_B_REQ, ssc::uds::ID_B_RESP, FW_VERSION, 0};

static bool can_tx(void *, uint16_t id, const uint8_t *d, uint8_t n) {
  if (!can_ok) return false;
  struct can_frame f;
  f.can_id = id;
  f.can_dlc = n;
  memcpy(f.data, d, n);
  return mcp.sendMessage(&f) == MCP2515::ERROR_OK;
}
static void ser_tx(void *, const uint8_t *d, size_t n) { Serial.write(d, n); }
static uint32_t us_now(void *) { return micros(); }
static void IRAM_ATTR on_wd_edge() { wd_edges++; }

// The health check a trial image must pass before it is committed. Minimal on purpose and stated: the CAN controller answers, the
// settings store opens, there is heap left. A real ECU runs its self-test suite here.
static bool health_ok() {
#ifdef OTA_UNHEALTHY
  return false;
#else
  Preferences p;
  bool store = p.begin("ssb", true);
  if (store) p.end();
  return can_ok && store && ESP.getFreeHeap() > 50000;
#endif
}

void setup() {
  Serial.setRxBufferSize(4096);
  Serial.begin(board::LINK_BAUD);
  const char *xtal = "none";
  can_ok = board::mcp_init(mcp, &xtal);
  pinMode(board::PIN_WD_LINE, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(board::PIN_WD_LINE), on_wd_edge, RISING);
  snprintf(fw, sizeof fw, "SafetyNode " FW_VERSION " (B) CAN %s", can_ok ? xtal : "FAILED - PiL only");
  uds_node.serial = (uint32_t)ESP.getEfuseMac();
  ota::boot_check(health_ok());   // a trial image that is unhealthy, or was reset before it confirmed, goes back to the old slot
  ssc::NodeIo io = {can_tx, ser_tx, us_now, nullptr};
  node.begin(io, fw);
  // v2.9.7, real reset: come back from ANY reset (EN pin, watchdog, brown-out) with the last configuration, already in
  // the latched safe state (STOP_IN_LANE, SAFETY_RESET), the way a real ECU boots with its calibration from flash.
  Preferences prefs;
  if (prefs.begin("ssb", true)) {
    stored_n = (uint8_t)prefs.getBytes("cfg", stored, sizeof stored);
    prefs.end();
  }
#ifndef SSC_NO_RESTORE   // build with -DSSC_NO_RESTORE for the negative test: a board that forgets its state on reset
  restored = stored_n > 0 && node.restore(stored, stored_n, (int64_t)millis());
#endif
  if (restored) strncat(fw, " restored", sizeof fw - strlen(fw) - 1);   // the hello says so
}

// Persist a new configuration only when it differs from the stored one (byte 0, warm/cold start, is not kept: a boot
// is always cold), so a test campaign writes flash once, not once per scenario.
static void persist_config() {
  if (node.config_seq() == saved_seq) return;
  saved_seq = node.config_seq();
  uint8_t n;
  const uint8_t *b = node.config_blob(&n);
  if (n == stored_n && memcmp(b + 1, stored + 1, n - 1) == 0) return;
  Preferences prefs;
  if (!prefs.begin("ssb", false)) return;
  prefs.putBytes("cfg", b, n);
  prefs.end();
  memcpy(stored, b, n);
  stored_n = n;
}

void loop() {
  uint8_t buf[256];
  int n;
  while ((n = Serial.available()) > 0) {
    n = Serial.read(buf, n < (int)sizeof buf ? n : (int)sizeof buf);
    for (int i = 0; i < n; i++) {
      if (!ota_parser.push(buf[i])) continue;
      if (ota_parser.type == 'R') pc_session = true;   // the PC started a run: no update until the next reset
      else if (ota_parser.type == 'U') ota::handle(ota_parser.payload, ota_parser.len, node.active() && pc_session, FW_VERSION);
    }
    node.feed(buf, (size_t)n, (int64_t)millis());
  }
  persist_config();
  ota::confirm_when_stable(millis(), health_ok(), FW_VERSION);
  if (wd_edges) {
    noInterrupts();
    uint32_t k = wd_edges;
    wd_edges = 0;
    interrupts();
    node.gpio_kicks((int)k, (int64_t)millis());
  }
  if (can_ok && millis() - last_flags_ms >= 5) {
    last_flags_ms = millis();
    uint8_t e = mcp.getErrorFlags();
    // v2.9.3: a CAN controller that has left normal mode (reset by a supply/SPI glitch) cannot send the actuator
    // commands: that is an actuator-bus fault too. Report it and re-initialise. Debounced: a lone SPI glitch is not one.
#ifndef SSC_MODE_CHECK
#define SSC_MODE_CHECK 1
#endif
    bool glitch;
    bool left_normal = SSC_MODE_CHECK && board::mcp_opmode_confirmed(&glitch) != 0;
    node.set_bus_fault(left_normal || (e & (MCP2515::EFLG_TXBO | MCP2515::EFLG_TXEP)) != 0);
    if (left_normal) {
      const char *x = "none";
      board::mcp_init(mcp, &x);
    }
    struct can_frame f;
    while (mcp.readMessage(&f) == MCP2515::ERROR_OK) {
      // B consumes no safety input from CAN. It only answers bench-health UDS requests (standard IDs 0x7E2 / 0x7DF).
      uint8_t out[8];
      uint8_t k = (f.can_id & CAN_EFF_FLAG) ? 0 : ssc::uds::respond(uds_node, (uint16_t)(f.can_id & 0x7FF), f.data, f.can_dlc, out);
      if (k) can_tx(nullptr, uds_node.resp_id, out, k);
    }
  }
  node.poll((int64_t)millis());
}
