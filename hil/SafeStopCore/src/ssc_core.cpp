// SafeStopCore implementation. Line-by-line port of ssb/safety.py and ssb/e2e.py; see ssc_core.h for the port rules.
#include "ssc_core.h"

#include <math.h>
#include <string.h>

namespace ssc {

static const int RANK[7] = {0, 0, 1, 2, 3, 4, 4};
static inline bool latched(State s) { return s == PULL_OVER || s == STOP_IN_LANE || s == BRAKE_ONLY_STOP || s == BACKUP_BRAKE_STOP; }
static inline bool e2e_valid(E2EStatus s) { return s == E_OK || s == E_OK_SOME_LOST; }
static inline bool e2e_error(E2EStatus s) { return s == E_WRONG_CRC || s == E_REPEATED || s == E_WRONG_SEQUENCE; }
static inline double pymax(double a, double b) { return b > a ? b : a; }  // Python max(a, b)
static inline double pymin(double a, double b) { return b < a ? b : a; }  // Python min(a, b)
static inline int64_t pymod(int64_t a, int64_t m) { int64_t r = a % m; return r < 0 ? r + m : r; }
static const double RAD_TO_DEG = 180.0 / 3.14159265358979323846;  // = CPython's math.degrees factor
static inline uint8_t qa_answer(uint8_t ch) { return (uint8_t)((ch * 31 + 7) & 0xFF); }

void config_from_array(Config &c, const double *v) {
  double *dst = &c.cmd_timeout_ms;  // the struct is N_CONFIG doubles in order
  for (int i = 0; i < N_CONFIG; i++) dst[i] = v[i];
}

uint16_t crc16_ccitt(const uint8_t *d, size_t n, uint16_t crc) {
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)(d[i] << 8);
    for (int k = 0; k < 8; k++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

uint8_t crc8_h2f(const uint8_t *d, size_t n, uint8_t crc) {
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int k = 0; k < 8; k++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x2F) : (uint8_t)(crc << 1);
  }
  return crc ^ 0xFF;
}

// ---- Controller ---------------------------------------------------------------------------------------------------------
void Controller::init(const Config &cfg, uint32_t defects, bool warm_start) {
  cfg_ = cfg;
  p_ = cfg;
  defects_ = defects;
  if (has(M_LONG_TIMEOUT)) p_.cmd_timeout_ms = 1000;
  rx_has_last_ = false;
  rx_last_ = 0;
  crc_fails_ = 0;
  win_n_ = 0;
  win_head_ = 0;
  sm_ = SM_NODATA;
  max_err_valid_ = has(M_E2E_LENIENT) ? 6 : 2;
  bad_run_ = 0;
  state = warm_start ? NORMAL : INIT;
  if (warm_start) {  // E2EStateMachine.preset_valid()
    for (int i = 0; i < 6; i++) win_[i] = E_OK;
    win_n_ = 6;
    sm_ = SM_VALID;
  }
  cause = C_NONE;
  has_fault = false;
  t_fault = 0;
  last_valid_ms_ = 0;
  valid_since_init_ = 0;
  cmd_accel_ = cmd_steer_ = cmd_speed_req_ = 0.0;
  cmd_perception_ = 2;
  cmd_odd_exit_ = false;
  has_last_steer_ = false;
  last_steer_ = 0.0;
  last_cmd_t_ = 0;
  last_accel_ = 0.0;
  env_active_ = false;
  env_since_ = 0;
  stale_run_ = 0;
  last_kick_ = 0;
  early_run_ = 0;
  n_challenges_ = 0;
  challenge = 0x5A;
  qa_fail_run_ = 0;
  steer_mis_active_ = brake_mis_active_ = healthy_perc_active_ = false;
  steer_mis_since_ = brake_mis_since_ = healthy_perc_since_ = 0;
  decel_now_ = 0.0;
  out_a = out_s = 0.0;
  out_backup = false;
  out_prev_steer_ = 0.0;
  act_counter_ = 0;
  status_counter_ = 0;
  rejected = 0;
  fault_active_t_ = -1000000000LL;
  release_rejected = 0;
  powered = true;
  mrm_pull_over = false;
  tx_ok = true;
  psi_mrm_ = 0.0;
}

void Controller::escalate(int64_t t, State s, Cause c) {
  fault_active_t_ = t;
  if (RANK[s] > RANK[state] || (state == INIT && s != NORMAL)) {
    if (!has_fault) {
      has_fault = true;
      t_fault = t;
      cause = c;
    }
    state = s;
    mrm_pull_over = (s == PULL_OVER);
  }
}

double Controller::steer_rate_limit(double v) const {
  double lim = pymax(p_.steer_rate_min_dps, p_.steer_rate_max_dps - p_.steer_rate_slope * v * 3.6);
  return lim + (has(M_ENVELOPE_OFF_BY_ONE) ? 5.0 : 0.0);
}

void Controller::kick(int64_t t) {
  if (has(M_NO_WATCHDOG) || !powered) return;
  if ((double)(t - last_kick_) < p_.wd_window_min_ms) {
    early_run_ += 1;
    if (early_run_ >= p_.wd_early_kicks) escalate(t, STOP_IN_LANE, C_WATCHDOG_EARLY);
  } else {
    early_run_ = 0;
  }
  last_kick_ = t;
}

bool Controller::release(int64_t t, double v) {
  bool ok_conditions = v == 0.0 && (double)(t - fault_active_t_) >= p_.release_clear_ms;
  if (latched(state) && (has(M_RELEASE_UNCONDITIONAL) || ok_conditions)) {
    state = INIT;
    valid_since_init_ = 0;
    mrm_pull_over = false;
    decel_now_ = 0.0;
    return true;
  }
  release_rejected += 1;
  return false;
}

void Controller::brownout(int64_t t, bool on) {
  if (on && powered) {
    powered = false;
  } else if (!on && !powered) {
    uint32_t keep = release_rejected;
    init(cfg_, defects_, false);
    release_rejected = keep;
    last_kick_ = t;
    last_valid_ms_ = t;
    escalate(t, STOP_IN_LANE, C_SAFETY_RESET);
  }
}

// v2.6 "explained gaps": a counter jump covered by the CRC failures just before it is not a second error
// (= ssb.e2e._Receiver with explain_gaps=True, the default since v2.6; max_delta 2).
E2EStatus Controller::p5_check(const uint8_t *f, int len, const uint8_t **payload) {
  if (len < 3) {
    crc_fails_++;
    return E_WRONG_CRC;
  }
  uint8_t counter = f[2];
  const uint8_t id[2] = {(uint8_t)(DATA_ID & 0xFF), (uint8_t)(DATA_ID >> 8)};
  uint16_t crc = crc16_ccitt(&counter, 1);
  crc = crc16_ccitt(f + 3, (size_t)(len - 3), crc);
  crc = crc16_ccitt(id, 2, crc);
  if (crc != (uint16_t)(f[0] | (f[1] << 8))) {
    crc_fails_++;
    return E_WRONG_CRC;
  }
  *payload = f + 3;
  int explained = crc_fails_;
  crc_fails_ = 0;
  if (!rx_has_last_) {
    rx_has_last_ = true;
    rx_last_ = counter;
    return E_OK;
  }
  uint8_t delta = (uint8_t)(counter - rx_last_);
  if (delta == 0) return E_REPEATED;
  rx_last_ = counter;
  if (delta == 1) return E_OK;
  return delta <= 2 + explained ? E_OK_SOME_LOST : E_WRONG_SEQUENCE;
}

SmState Controller::sm_update(E2EStatus s) {
  if (s == E_NO_NEW_DATA) return sm_;
  if (win_n_ < 6) {
    win_[win_n_++] = s;
  } else {
    memmove(win_, win_ + 1, 5);
    win_[5] = s;
  }
  int ok = 0, err = 0;
  for (int i = 0; i < win_n_; i++) {
    ok += e2e_valid((E2EStatus)win_[i]);
    err += e2e_error((E2EStatus)win_[i]);
  }
  if (sm_ == SM_NODATA) sm_ = SM_INIT;
  if (sm_ == SM_INIT) {
    if (ok >= 2 && err <= 1) sm_ = SM_VALID;
    else if (err > 1) sm_ = SM_INVALID;
  } else if (sm_ == SM_VALID) {
    if (err > max_err_valid_) sm_ = SM_INVALID;
  } else if (sm_ == SM_INVALID) {
    if (win_n_ >= 4) {
      bool all = true;
      for (int i = win_n_ - 4; i < win_n_; i++) all = all && e2e_valid((E2EStatus)win_[i]);
      if (all) sm_ = SM_VALID;
    }
  }
  return sm_;
}

void Controller::envelope(int64_t t, double &accel, double &steer, double &speed_req, double v) {
  bool bad = false;
  if (has_last_steer_ && t > last_cmd_t_) {
    double dt = pymax(0.02, (double)(t - last_cmd_t_) / 1000.0);
    double rate = (steer - last_steer_) / dt;
    double lim = steer_rate_limit(v);
    if (fabs(rate) > lim) {
      steer = last_steer_ + copysign(lim * dt, rate);
      bad = true;
    }
  }
  has_last_steer_ = true;
  last_steer_ = steer;
  last_cmd_t_ = t;
  if (speed_req > p_.odd_max_kmh / 3.6) {
    speed_req = p_.odd_max_kmh / 3.6;
    bad = true;
  }
  if (!(p_.min_accel <= accel && accel <= p_.max_accel)) {
    accel = pymax(p_.min_accel, pymin(p_.max_accel, accel));
    bad = true;
  }
  double jerk = (accel - last_accel_) / 0.02;
  if (fabs(jerk) > p_.max_jerk) accel = last_accel_ + copysign(p_.max_jerk * 0.02, jerk);
  last_accel_ = accel;
  if (v > p_.odd_max_kmh / 3.6 + 0.3 && accel > -0.5) accel = -0.5;
  if (bad) {
    if (!env_active_) {
      env_active_ = true;
      env_since_ = t;
    }
    if ((double)(t - env_since_) >= p_.env_debounce_ms) escalate(t, STOP_IN_LANE, C_ENVELOPE);
  } else {
    env_active_ = false;
  }
}

void Controller::perception(int64_t t) {
  if ((double)(t - last_valid_ms_) > p_.cmd_timeout_ms) return;
  if (cmd_perception_ == 0) {
    escalate(t, STOP_IN_LANE, C_PERCEPTION_LOST);
  } else if (cmd_odd_exit_) {
    escalate(t, PULL_OVER, C_ODD_EXIT);
  } else if (cmd_perception_ == 1) {
    escalate(t, DEGRADED, C_PERCEPTION_DEGRADED);
    healthy_perc_active_ = false;
  } else if (state == DEGRADED) {
    if (!healthy_perc_active_) {
      healthy_perc_active_ = true;
      healthy_perc_since_ = t;
    }
    if ((double)(t - healthy_perc_since_) >= p_.degraded_recover_ms) state = NORMAL;
  }
}

void Controller::actuator_checks(int64_t t, const Feedback &fb) {
  if (fabs(out_s - fb.delta) > p_.steer_mismatch_deg) {
    if (!steer_mis_active_) {
      steer_mis_active_ = true;
      steer_mis_since_ = t;
    }
    if ((double)(t - steer_mis_since_) >= p_.steer_mismatch_ms) escalate(t, BRAKE_ONLY_STOP, C_STEER_ACTUATOR);
  } else {
    steer_mis_active_ = false;
  }
  double a_cmd = out_a;
  double a_brake = has(M_NO_GRADE_COMPENSATION) ? fb.a : fb.a + fb.grade_accel;
  if (a_cmd < -1.0 && fb.v > 0.5 && a_brake > a_cmd * p_.brake_min_ratio) {
    if (!brake_mis_active_) {
      brake_mis_active_ = true;
      brake_mis_since_ = t;
    }
    if ((double)(t - brake_mis_since_) >= p_.brake_mismatch_ms) escalate(t, BACKUP_BRAKE_STOP, C_BRAKE_ACTUATOR);
  } else {
    brake_mis_active_ = false;
  }
}

double Controller::ramp(double target, double jerk) {
  decel_now_ = pymin(target, decel_now_ + jerk * CYCLE_MS / 1000);
  return decel_now_ > 0 ? -decel_now_ : 0.0;
}

void Controller::output(const Feedback &fb) {
  const double v = fb.v;
  const double steer_lim = steer_rate_limit(v) * CYCLE_MS / 1000;
  auto toward = [&](double target) {
    return out_prev_steer_ + pymax(-steer_lim, pymin(steer_lim, target - out_prev_steer_));
  };
  double a, s;
  bool b = false;
  switch (state) {
    case INIT: a = -1.0; s = 0.0; break;
    case NORMAL: decel_now_ = 0.0; a = cmd_accel_; s = cmd_steer_; break;
    case DEGRADED: {
      double cap = p_.degraded_cap_kmh / 3.6;
      double x = has(M_NO_SPEED_CAP) ? cmd_accel_ : pymin(cmd_accel_, 0.8 * (cap - v));
      a = pymax(-2.0, x);
      s = cmd_steer_;
      break;
    }
    case PULL_OVER: a = pymin(0.0, cmd_accel_); s = cmd_steer_; break;
    default: {
      if (v == 0.0) {
        a = -1.0;
        s = state != BRAKE_ONLY_STOP ? toward(0.0) : fb.delta;
        b = state == BACKUP_BRAKE_STOP;
        break;
      }
      if (!has(M_NO_HEADING_HOLD)) psi_mrm_ += fb.yaw_rate * CYCLE_MS / 1000;
      double hold = has(M_NO_HEADING_HOLD) ? 0.0 : pymax(-6.0, pymin(6.0, -(psi_mrm_ * RAD_TO_DEG) * 2.0));
      if (state == STOP_IN_LANE) {
        a = ramp(p_.mrm_decel, p_.mrm_jerk);
        s = toward(hold);
      } else if (state == BRAKE_ONLY_STOP) {
        a = ramp(p_.brake_only_decel, p_.emergency_jerk);
        s = fb.delta;
      } else {  // BACKUP_BRAKE_STOP
        a = ramp(p_.mrm_decel, p_.mrm_jerk);
        s = toward(hold);
        b = true;
      }
    }
  }
  out_a = a;
  out_s = s;
  out_backup = b;
}

static int16_t to_i16(double x) {
  double r = nearbyint(x);  // Python round(): half to even
  if (r > 32767) r = 32767;
  if (r < -32768) r = -32768;
  return (int16_t)r;
}

int Controller::cycle(int64_t t, const Frame *frames, int n, const Feedback &fb, uint8_t act[7]) {
  if (!powered) return 0;
  const double v = fb.v;
  if (t % 20 == 10) {
    challenge = (uint8_t)((challenge * 73 + 41) & 0xFF);
    if (n_challenges_ < 3) {
      challenges_[n_challenges_++] = challenge;
    } else {
      challenges_[0] = challenges_[1];
      challenges_[1] = challenges_[2];
      challenges_[2] = challenge;
    }
  }
  for (int i = 0; i < n; i++) {
    const Frame &f = frames[i];
    if (f.id != CMD_ID) continue;
    const uint8_t *pl = 0;
    E2EStatus status = p5_check(f.data, f.len, &pl);
    SmState sm = sm_update(status);
    if (e2e_error(status)) {
      rejected += 1;
      bad_run_ += 1;
    } else if (e2e_valid(status)) {
      bad_run_ = 0;
    }
    bool e2e_bad = has(M_E2E_RUN_LENGTH) ? (bad_run_ >= 3) : (sm == SM_INVALID);
    if (e2e_bad) escalate(t, STOP_IN_LANE, C_E2E_INVALID);
    if (!e2e_valid(status) || !(sm == SM_VALID || sm == SM_INIT)) continue;
    if (f.len != 14) continue;  // Python would raise on a wrong-length payload with a valid CRC; never happens in scenarios
    uint16_t ts = (uint16_t)(pl[0] | (pl[1] << 8));
    int16_t ra = (int16_t)(pl[2] | (pl[3] << 8)), rs = (int16_t)(pl[4] | (pl[5] << 8));
    uint16_t rv = (uint16_t)(pl[6] | (pl[7] << 8));
    uint8_t qa = pl[8], perc = pl[9], flags = pl[10];
    double accel = ra / 100.0, steer = rs / 100.0, speed_req = rv / 100.0;
    int64_t age = pymod(t - ts, 65536);
    if (!has(M_NO_FRESHNESS) && (double)age > p_.max_age_ms) {
      stale_run_ += 1;
      if (stale_run_ >= 3) escalate(t, STOP_IN_LANE, C_STALE_DATA);
      continue;
    }
    stale_run_ = 0;
    if (!has(M_NO_QA)) {
      bool ok = n_challenges_ == 0;
      for (int k = 0; k < n_challenges_; k++) ok = ok || qa == qa_answer(challenges_[k]);
      if (ok) {
        qa_fail_run_ = 0;
      } else {
        qa_fail_run_ += 1;
        if (qa_fail_run_ >= 3) escalate(t, STOP_IN_LANE, C_WATCHDOG_QA);
      }
    }
    envelope(t, accel, steer, speed_req, v);
    cmd_accel_ = accel;
    cmd_steer_ = steer;
    cmd_speed_req_ = speed_req;
    cmd_perception_ = perc;
    cmd_odd_exit_ = (flags & 1) != 0;
    last_valid_ms_ = t;
    valid_since_init_ += 1;
  }

  if ((double)(t - last_valid_ms_) > p_.cmd_timeout_ms) escalate(t, STOP_IN_LANE, C_TIMEOUT);
  if (!has(M_NO_WATCHDOG) && (double)(t - last_kick_) > p_.wd_window_max_ms) escalate(t, STOP_IN_LANE, C_WATCHDOG_LATE);
  if (!tx_ok) escalate(t, STOP_IN_LANE, C_ACT_BUS_OFF);
  perception(t);
  if (!has(M_NO_ACTUATOR_CHECK)) actuator_checks(t, fb);
  if (state == INIT) {
    if (valid_since_init_ >= 2 || has(M_STARTUP_MOVES)) {
      if ((double)(t - last_valid_ms_) <= p_.cmd_timeout_ms || has(M_STARTUP_MOVES)) state = NORMAL;
    }
  }
  if (latched(state) && has(M_NO_LATCH)) {
    bool healthy = (t - last_valid_ms_) <= 40 && sm_ == SM_VALID && (double)(t - last_kick_) <= p_.wd_window_max_ms;
    if (healthy && state == STOP_IN_LANE) state = NORMAL;
  }
  output(fb);
  out_prev_steer_ = out_s;
  // Profile 2: CRC-8 0x2F over (counter, payload, data ID picked from 0x10..0x1F by the counter)
  uint8_t c = act_counter_ & 0x0F;
  int16_t ia = to_i16(out_a * 100), is = to_i16(out_s * 100);
  uint8_t buf[7] = {c, (uint8_t)(ia & 0xFF), (uint8_t)((uint16_t)ia >> 8), (uint8_t)(is & 0xFF), (uint8_t)((uint16_t)is >> 8),
                    (uint8_t)(out_backup ? 1 : 0), (uint8_t)(0x10 + c)};
  act[0] = crc8_h2f(buf, 7);
  act[1] = c;
  memcpy(act + 2, buf + 1, 5);
  act_counter_ = (uint8_t)((act_counter_ + 1) % 16);
  return 7;
}

static_assert(OFF < 8, "SAF_Status packs the state in 3 bits");
static_assert(C_SAFETY_RESET < 16, "SAF_Status packs the cause in 4 bits");

void Controller::status_frame(uint8_t out[8]) {
  uint8_t st = powered ? (uint8_t)state : (uint8_t)OFF;
  out[1] = status_counter_;
  status_counter_ = (uint8_t)(status_counter_ + 1);
  out[2] = (uint8_t)((st & 0x07) | (mrm_pull_over ? 0x08 : 0) | (((uint8_t)cause & 0x0F) << 4));
  out[3] = challenge;
  int16_t a = to_i16(pymax(-327.0, pymin(327.0, out_a)) * 100), s = to_i16(pymax(-327.0, pymin(327.0, out_s)) * 100);
  out[4] = (uint8_t)(a & 0xFF);
  out[5] = (uint8_t)((uint16_t)a >> 8);
  out[6] = (uint8_t)(s & 0xFF);
  out[7] = (uint8_t)((uint16_t)s >> 8);
  uint8_t buf[9];
  memcpy(buf, out + 1, 7);
  buf[7] = (uint8_t)(STATUS_ID & 0xFF);
  buf[8] = (uint8_t)(STATUS_ID >> 8);
  out[0] = crc8_h2f(buf, 9);
}

// ---- framing -------------------------------------------------------------------------------------------------------------
size_t frame_msg(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out) {
  out[0] = SYNC;
  out[1] = type;
  out[2] = len;
  if (len) memcpy(out + 3, payload, len);
  out[3 + len] = crc8_h2f(out + 1, (size_t)len + 2);
  return (size_t)len + 4;
}

bool Parser::push(uint8_t b) {
  switch (pos_) {
    case 0:
      if (b == SYNC) pos_ = 1;
      return false;
    case 1: type = b; pos_ = 2; return false;
    case 2:
      len = b;
      need_ = b;
      pos_ = 3;
      return false;
    default:
      if (pos_ - 3 < need_) {
        payload[pos_ - 3] = b;
        pos_++;
        return false;
      }
      pos_ = 0;
      {
        uint8_t hdr[2] = {type, len};
        uint8_t crc = 0;
        // crc8_h2f over type, len, payload (same as frame_msg); computed in two steps without a final-XOR break
        uint8_t tmp[257];
        memcpy(tmp, hdr, 2);
        memcpy(tmp + 2, payload, len);
        crc = crc8_h2f(tmp, (size_t)len + 2);
        if (crc != b) {
          errors++;
          if (b == SYNC) pos_ = 1;  // resynchronise
          return false;
        }
      }
      return true;
  }
}

// ---- node ----------------------------------------------------------------------------------------------------------------
static double rd_f64(const uint8_t *p) {
  double d;
  memcpy(&d, p, 8);  // both targets are little-endian
  return d;
}
static void wr_u32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

void Node::begin(const NodeIo &io, const char *fw) {
  io_ = io;
  fw_ = fw;
  active_ = false;
  bus_fault_ = false;
  last_hello_ = -100000;
  kick_src_ = 0;
  bus_monitor_ = false;
  n_pending_ = 0;
  n_blob_ = 0;
  config_seq_ = 0;
}

void Node::send(uint8_t type, const uint8_t *p, uint8_t n) {
  uint8_t buf[260];
  size_t k = frame_msg(type, p, n, buf);
  io_.ser_tx(io_.ctx, buf, k);
}

void Node::emit_can(uint16_t id, const uint8_t *d, uint8_t n) {
  if (io_.can_tx && !io_.can_tx(io_.ctx, id, d, n)) can_tx_fail_++;
  uint8_t m[3 + 8] = {(uint8_t)(id & 0xFF), (uint8_t)(id >> 8), n};
  memcpy(m + 3, d, n);
  send('M', m, (uint8_t)(3 + n));
}

void Node::feed(const uint8_t *d, size_t n, int64_t now_ms) {
  for (size_t i = 0; i < n; i++)
    if (parser_.push(d[i])) handle(now_ms);
}

void Node::gpio_kicks(int n, int64_t now_ms) {
  if (!active_ || kick_src_ != 1) return;
  for (int i = 0; i < n; i++) sc.kick(now_ms - t0_);
}

bool Node::start(const uint8_t *p, uint8_t n, int64_t now_ms) {
  if (n < 7 || p[6] != N_CONFIG || n < 7 + 8 * N_CONFIG) return false;
  double v[N_CONFIG];
  for (int i = 0; i < N_CONFIG; i++) v[i] = rd_f64(p + 7 + 8 * i);
  Config cfg;
  config_from_array(cfg, v);
  uint32_t defects = (uint32_t)p[1] | ((uint32_t)p[2] << 8) | ((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 24);
  sc.init(cfg, defects, p[0] == 1);
  kick_src_ = p[5] & 1;
  bus_monitor_ = (p[5] >> 1) & 1;  // off for a single board: nobody would ACK, so error passive is expected
  t0_ = now_ms;
  next_cycle_ = 0;
  n_pending_ = 0;
  memset(&fb_, 0, sizeof fb_);
  power_ok_ = pc_tx_ok_ = true;
  last_release_ = false;
  overflow_ = can_tx_fail_ = cycles_ = max_exec_us_ = 0;
  max_late_ms_ = 0;
  active_ = true;
  return true;
}

bool Node::restore(const uint8_t *blob, uint8_t n, int64_t now_ms) {
  if (n != 7 + 8 * N_CONFIG) return false;
  uint8_t p[7 + 8 * N_CONFIG];
  memcpy(p, blob, n);
  p[0] = 2;   // cold
  if (!start(p, n, now_ms)) return false;
  sc.brownout(0, true);    // the reset: unpowered...
  sc.brownout(0, false);   // ...and back: cold init, then latched STOP_IN_LANE with cause SAFETY_RESET (SG6)
  memcpy(blob_, blob, n);
  n_blob_ = n;
  return true;
}

void Node::handle(int64_t now_ms) {
  const uint8_t *p = parser_.payload;
  const uint8_t n = parser_.len;
  switch (parser_.type) {
    case 'R': {
      if (!start(p, n, now_ms)) return;
      n_blob_ = (uint8_t)(7 + 8 * N_CONFIG);
      memcpy(blob_, p, n_blob_);
      config_seq_++;
      uint8_t ack[1] = {(uint8_t)sc.state};
      send('A', ack, 1);
      return;
    }
    default: break;
  }
  if (!active_) return;
  const int64_t t = now_ms - t0_;
  switch (parser_.type) {
    case 'P':
      if (n_pending_ < 16 && n <= 16) {
        pending_[n_pending_].id = CMD_ID;
        pending_[n_pending_].len = n;
        memcpy(pending_[n_pending_].data, p, n);
        n_pending_++;
      } else {
        overflow_++;
      }
      break;
    case 'K':
      if (n >= 1 && kick_src_ == 0)
        for (int i = 0; i < p[0]; i++) sc.kick(t);
      break;
    case 'F':
      if (n >= 40) {
        fb_.v = rd_f64(p);
        fb_.a = rd_f64(p + 8);
        fb_.delta = rd_f64(p + 16);
        fb_.yaw_rate = rd_f64(p + 24);
        fb_.grade_accel = rd_f64(p + 32);
      }
      break;
    case 'C':
      if (n >= 1) {
        power_ok_ = p[0] & 1;
        sc.brownout(t, !power_ok_);
        pc_tx_ok_ = (p[0] >> 1) & 1;
        bool rel = (p[0] >> 2) & 1;
        if (rel && !last_release_) sc.release(t, fb_.v);
        last_release_ = rel;
      }
      break;
    default: break;
  }
}

void Node::poll(int64_t now_ms) {
  // v2.9.7: a board restored after a real reset is active at once, so it says hello while active too (less often)
  if (now_ms - last_hello_ >= (active_ ? 1000 : 500)) {
    last_hello_ = now_ms;
    send('H', (const uint8_t *)fw_, (uint8_t)strlen(fw_));
  }
  if (!active_) return;
  const int64_t t = now_ms - t0_;
  while (next_cycle_ <= t) {
    sc.tx_ok = pc_tx_ok_ && !(bus_monitor_ && bus_fault_);
    uint32_t u0 = io_.micros ? io_.micros(io_.ctx) : 0;
    uint8_t act[7], st[8];
    int k = sc.cycle(next_cycle_, pending_, n_pending_, fb_, act);
    n_pending_ = 0;
    uint32_t exec = io_.micros ? io_.micros(io_.ctx) - u0 : 0;
    if (exec > max_exec_us_) max_exec_us_ = exec;
    if (k) emit_can(ACT_ID, act, 7);
    sc.status_frame(st);
    emit_can(STATUS_ID, st, 8);
    if (t - next_cycle_ > max_late_ms_) max_late_ms_ = t - next_cycle_;
    cycles_++;
    next_cycle_ += CYCLE_MS;
    if (cycles_ % 10 == 0) {
      uint8_t d[23];
      wr_u32(d, cycles_);
      wr_u32(d + 4, max_exec_us_);
      wr_u32(d + 8, (uint32_t)max_late_ms_);
      wr_u32(d + 12, can_tx_fail_);
      wr_u32(d + 16, overflow_);
      d[20] = (uint8_t)(parser_.errors > 255 ? 255 : parser_.errors);
      d[21] = bus_fault_ ? 1 : 0;
      d[22] = (uint8_t)(kick_src_ | (bus_monitor_ ? 2 : 0));
      send('D', d, sizeof d);
    }
  }
}

}  // namespace ssc
