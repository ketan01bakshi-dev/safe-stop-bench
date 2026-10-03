// SafeStopCore: the reference safety controller (ssb/safety.py) ported to portable C++.
//
// Same source builds for the ESP32-S3 (Arduino, the HiL target) and for the PC (a DLL that the bench drives through
// ctypes, so the port is proven back-to-back against the Python reference BEFORE it goes on the chip).
// No Arduino calls, no heap: everything is fixed-size. Times are integer milliseconds since the bench reset.
//
// Port rules (so results are identical to Python): doubles everywhere Python uses floats, the same order of
// operations, round-half-to-even (nearbyint) where Python calls round(), and Python's non-negative modulo.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace ssc {

enum State : uint8_t { INIT = 0, NORMAL, DEGRADED, PULL_OVER, STOP_IN_LANE, BRAKE_ONLY_STOP, BACKUP_BRAKE_STOP, OFF };
// Order = ssb.canio.CAUSES (also the SAF_Cause value on CAN).
enum Cause : uint8_t {
  C_NONE = 0, C_TIMEOUT, C_E2E_INVALID, C_STALE_DATA, C_WATCHDOG_LATE, C_WATCHDOG_EARLY, C_WATCHDOG_QA, C_ENVELOPE,
  C_STEER_ACTUATOR, C_BRAKE_ACTUATOR, C_ACT_BUS_OFF, C_PERCEPTION_DEGRADED, C_ODD_EXIT, C_PERCEPTION_LOST, C_SAFETY_RESET
};
// Seeded bugs; bit k = k-th entry of ssb.safety.MUTANTS.
enum Mutant : uint32_t {
  M_NO_LATCH = 1u << 0, M_NO_WATCHDOG = 1u << 1, M_LONG_TIMEOUT = 1u << 2, M_ENVELOPE_OFF_BY_ONE = 1u << 3,
  M_E2E_LENIENT = 1u << 4, M_E2E_RUN_LENGTH = 1u << 5, M_NO_FRESHNESS = 1u << 6, M_NO_QA = 1u << 7,
  M_NO_ACTUATOR_CHECK = 1u << 8, M_RELEASE_UNCONDITIONAL = 1u << 9, M_STARTUP_MOVES = 1u << 10,
  M_NO_SPEED_CAP = 1u << 11, M_NO_HEADING_HOLD = 1u << 12, M_NO_GRADE_COMPENSATION = 1u << 13
};

// Calibration, downloaded by the bench at every reset (order = ssb.native.CFG_KEYS).
struct Config {
  double cmd_timeout_ms, max_age_ms, wd_window_min_ms, wd_window_max_ms, wd_early_kicks, steer_rate_max_dps,
      steer_rate_slope, steer_rate_min_dps, odd_max_kmh, max_accel, min_accel, max_jerk, env_debounce_ms,
      steer_mismatch_deg, steer_mismatch_ms, brake_min_ratio, brake_mismatch_ms, degraded_cap_kmh, degraded_recover_ms,
      mrm_decel, mrm_jerk, brake_only_decel, emergency_jerk, release_clear_ms;
};
const int N_CONFIG = 24;
void config_from_array(Config &c, const double *v);

const uint16_t CMD_ID = 0x100, ACT_ID = 0x200, STATUS_ID = 0x201;
const int CYCLE_MS = 10;
const uint16_t DATA_ID = 0x1234;

uint16_t crc16_ccitt(const uint8_t *d, size_t n, uint16_t crc = 0xFFFF);
uint8_t crc8_h2f(const uint8_t *d, size_t n, uint8_t crc = 0xFF);

enum E2EStatus : uint8_t { E_OK, E_OK_SOME_LOST, E_NO_NEW_DATA, E_WRONG_CRC, E_REPEATED, E_WRONG_SEQUENCE };
enum SmState : uint8_t { SM_NODATA, SM_INIT, SM_VALID, SM_INVALID };

struct Frame {
  uint16_t id;
  uint8_t len;
  uint8_t data[16];
};
struct Feedback {
  double v, a, delta, yaw_rate, grade_accel;
};

class Controller {
 public:
  void init(const Config &cfg, uint32_t defects, bool warm_start);
  void kick(int64_t t);
  bool release(int64_t t, double v);
  void brownout(int64_t t, bool on);
  // One 10 ms cycle. Writes the Profile 2 SAF_ActuatorCmd frame (7 bytes) to act and returns 7, or 0 if unpowered.
  int cycle(int64_t t, const Frame *frames, int n, const Feedback &fb, uint8_t act[7]);
  // SAF_Status as in dbc/safe_stop.dbc (8 bytes): state, cause, challenge, MRM request, accel out, steer out.
  void status_frame(uint8_t out[8]) const;

  // observable state (what ReferenceDUT reads from the Python controller)
  State state;
  Cause cause;
  bool has_fault;
  int64_t t_fault;
  uint8_t challenge;
  bool mrm_pull_over;
  double out_a, out_s;
  bool out_backup;
  bool powered;
  bool tx_ok;
  uint32_t rejected, release_rejected;

 private:
  void escalate(int64_t t, State s, Cause c);
  double steer_rate_limit(double v) const;
  void envelope(int64_t t, double &accel, double &steer, double &speed_req, double v);
  void perception(int64_t t);
  void actuator_checks(int64_t t, const Feedback &fb);
  double ramp(double target, double jerk);
  void output(const Feedback &fb);
  E2EStatus p5_check(const uint8_t *f, int len, const uint8_t **payload);
  SmState sm_update(E2EStatus s);
  bool has(uint32_t m) const { return (defects_ & m) != 0; }

  Config cfg_, p_;
  uint32_t defects_;
  // E2E Profile 5 receiver + windowed state machine (window 6)
  bool rx_has_last_;
  uint8_t rx_last_;
  uint8_t win_[6];
  int win_n_, win_head_;
  SmState sm_;
  int max_err_valid_;
  int bad_run_;
  // command
  int64_t last_valid_ms_;
  int valid_since_init_;
  double cmd_accel_, cmd_steer_, cmd_speed_req_;
  int cmd_perception_;
  bool cmd_odd_exit_;
  bool has_last_steer_;
  double last_steer_;
  int64_t last_cmd_t_;
  double last_accel_;
  bool env_active_;
  int64_t env_since_;
  int stale_run_;
  int64_t last_kick_;
  int early_run_;
  uint8_t challenges_[3];
  int n_challenges_;
  int qa_fail_run_;
  bool steer_mis_active_, brake_mis_active_, healthy_perc_active_;
  int64_t steer_mis_since_, brake_mis_since_, healthy_perc_since_;
  double decel_now_;
  double out_prev_steer_;
  uint8_t act_counter_;
  int64_t fault_active_t_;
  double psi_mrm_;
};

// ---- the bench link: framing used on USB serial between the PC and the boards ----------------------------------------
// [0xA5][type][len][payload: len bytes][crc8_h2f(type, len, payload)]
const uint8_t SYNC = 0xA5;
size_t frame_msg(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out);  // out needs len + 4 bytes

class Parser {
 public:
  Parser() : errors(0), pos_(0), need_(0) {}
  // Feed one byte; returns true when a complete, CRC-checked message is in type/len/payload.
  bool push(uint8_t b);
  uint8_t type, len;
  uint8_t payload[255];
  uint32_t errors;

 private:
  int pos_;
  uint8_t need_;
};

// ---- the safety node: what runs on board B, hardware-independent -------------------------------------------------------
// PC -> node:  'R' reset (mode, defects, options: bit0 GPIO kicks, bit1 bus monitoring; calibration)  'P' planner frame  'K' watchdog kicks
//              'F' feedback (5 x float64)  'C' control (power, tx ok, release)
// node -> PC:  'H' hello  'A' reset acknowledged  'M' a CAN frame the node sent (mirror)  'D' diagnostics
struct NodeIo {
  bool (*can_tx)(void *ctx, uint16_t id, const uint8_t *d, uint8_t n);  // may be null (no CAN on the board)
  void (*ser_tx)(void *ctx, const uint8_t *d, size_t n);
  uint32_t (*micros)(void *ctx);  // may be null (no execution-time measurement)
  void *ctx;
};

class Node {
 public:
  void begin(const NodeIo &io, const char *fw);
  void feed(const uint8_t *d, size_t n, int64_t now_ms);
  void poll(int64_t now_ms);
  void gpio_kicks(int n, int64_t now_ms);  // edges seen on the hardware watchdog line
  void set_bus_fault(bool f) { bus_fault_ = f; }
  bool active() const { return active_; }
  uint8_t kick_source() const { return kick_src_; }
  bool bus_monitor() const { return bus_monitor_; }
  Controller sc;

 private:
  void handle(int64_t now_ms);
  void send(uint8_t type, const uint8_t *p, uint8_t n);
  void emit_can(uint16_t id, const uint8_t *d, uint8_t n);
  NodeIo io_;
  const char *fw_;
  Parser parser_;
  bool active_, power_ok_, pc_tx_ok_, last_release_, bus_fault_, bus_monitor_;
  uint8_t kick_src_;
  int64_t t0_, next_cycle_, last_hello_;
  Frame pending_[16];
  int n_pending_;
  uint32_t overflow_, can_tx_fail_, cycles_;
  uint32_t max_exec_us_;
  int64_t max_late_ms_;
  Feedback fb_;
};

}  // namespace ssc
