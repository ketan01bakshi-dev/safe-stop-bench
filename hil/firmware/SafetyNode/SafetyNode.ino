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
#include <Preferences.h>

static MCP2515 mcp(board::PIN_CAN_CS);
static bool can_ok = false;
static ssc::Node node;
static volatile uint32_t wd_edges = 0;
static uint32_t last_flags_ms = 0;
static char fw[64];
static uint8_t stored[256], stored_n = 0;
static uint32_t saved_seq = 0;
static bool restored = false;

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

void setup() {
  Serial.setRxBufferSize(4096);
  Serial.begin(board::LINK_BAUD);
  const char *xtal = "none";
  can_ok = board::mcp_init(mcp, &xtal);
  pinMode(board::PIN_WD_LINE, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(board::PIN_WD_LINE), on_wd_edge, RISING);
  snprintf(fw, sizeof fw, "SafetyNode 2.6 (B) CAN %s", can_ok ? xtal : "FAILED - PiL only");
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
    node.feed(buf, (size_t)n, (int64_t)millis());
  }
  persist_config();
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
    }  // B consumes nothing from CAN; keep the receive buffers empty
  }
  node.poll((int64_t)millis());
}
