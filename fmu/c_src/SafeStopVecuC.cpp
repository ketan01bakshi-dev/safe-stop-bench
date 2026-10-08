// SafeStopVecuC: the safety-controller core (hil/SafeStopCore, C++) packaged as a COMPILED FMI 2.0 co-simulation FMU.
//
// This is what a supplier actually ships: a zip with modelDescription.xml and a native binaries/win64 DLL, no Python
// inside. It exposes exactly the same variables as fmu/SafeStopVecu.py (the PythonFMU reference), so the bench drives it
// through the same identity mapping, and an exact back-to-back against the Python reference must show no difference.
//
// Built by scripts/build_c_fmu.py, which also generates fmu_gen.h (value references, start values, bundled configs)
// and modelDescription.xml from ONE table, so the XML and the C code can't drift apart.
// No dynamic allocation in the step path; one heap block per instance (fmi2Instantiate / fmi2FreeInstance).
#include <stdio.h>
#include <string.h>

#include "fmi2Functions.h"
#include "fmu_gen.h"
#include "ssc_core.h"

using namespace ssc;

namespace {

struct Inst {
  fmi2CallbackFunctions cb;
  char name[64];
  bool init_mode, initialized;
  // parameters
  fmi2Boolean warm_start;
  fmi2Integer defects;
  char config[32];
  // inputs / outputs, indexed by value reference (one array per FMI type)
  fmi2Integer ints[N_INT];
  fmi2Real reals[N_REAL];
  fmi2Boolean bools[N_BOOL];
  // controller and the edge detectors on the event counters
  Controller sc;
  Frame pending[16];
  int n_pending;
  fmi2Integer rx_seen, kick_seen;
};

void log(Inst *x, fmi2Status s, const char *msg) {
  if (x && x->cb.logger) x->cb.logger(x->cb.componentEnvironment, x->name, s, "log", "%s", msg);
}

void set_defaults(Inst *x) {
  x->init_mode = x->initialized = false;
  x->warm_start = fmi2True;
  x->defects = 0;
  snprintf(x->config, sizeof x->config, "%s", "default");
  for (int i = 0; i < N_INT; i++) x->ints[i] = INT_START[i];
  for (int i = 0; i < N_REAL; i++) x->reals[i] = 0.0;
  for (int i = 0; i < N_BOOL; i++) x->bools[i] = BOOL_START[i];
  x->n_pending = 0;
}

// Same as SafeStopVecu._publish (Python): state OFF when unpowered, first cause, challenge, MRM, commanded outputs.
void publish(Inst *x) {
  const Controller &c = x->sc;
  x->ints[VR_SAF_State] = c.powered ? c.state : OFF;
  x->ints[VR_SAF_Cause] = c.cause;
  x->ints[VR_SAF_WdChallenge] = c.challenge;
  x->ints[VR_SAF_MrmRequest] = c.mrm_pull_over ? 1 : 0;
  x->reals[VR_SAF_AccelOut] = c.out_a;
  x->reals[VR_SAF_SteerOut] = c.out_s;
}

fmi2Status start(Inst *x) {
  const double *cfg = 0;
  for (int i = 0; i < N_CONFIGS; i++)
    if (strcmp(CONFIG_NAMES[i], x->config) == 0) cfg = CONFIG_VALUES[i];
  if (!cfg) {
    log(x, fmi2Error, "Bench_Config: unknown config name");
    return fmi2Error;
  }
  Config c;
  config_from_array(c, cfg);
  x->sc.init(c, (uint32_t)x->defects, x->warm_start != fmi2False);
  x->n_pending = 0;
  x->rx_seen = x->ints[VR_PLN_Command_RxCounter];
  x->kick_seen = x->ints[VR_PLN_WdKickCounter];
  publish(x);
  return fmi2OK;
}

void put(uint8_t *&p, int64_t v, int bytes) {  // little-endian, two's complement (struct.pack "<H/h/B" on raw values)
  for (int i = 0; i < bytes; i++) *p++ = (uint8_t)((uint64_t)v >> (8 * i));
}

// One 1 ms communication step at time t: same order as SafeStopVecu.do_step.
void step(Inst *x, int64_t t) {
  Controller &sc = x->sc;
  fmi2Integer *in = x->ints;  // inputs and outputs share this array (distinct value references)
  sc.brownout(t, x->bools[VR_Bench_PowerOk] == fmi2False);
  sc.tx_ok = x->bools[VR_Bench_TxOk] != fmi2False;
  for (int k = (int)((in[VR_PLN_WdKickCounter] - x->kick_seen) & 0xFF); k > 0; k--) sc.kick(t);
  x->kick_seen = in[VR_PLN_WdKickCounter];
  if (x->bools[VR_Bench_Release]) sc.release(t, x->reals[VR_VEH_Speed]);
  if (in[VR_PLN_Command_RxCounter] != x->rx_seen) {
    x->rx_seen = in[VR_PLN_Command_RxCounter];
    uint8_t d[14], *p = d;  // PLN_Command layout, dbc/safe_stop.dbc: <HBHhhHBBB
    put(p, in[VR_PLN_E2E_CRC], 2);
    put(p, in[VR_PLN_E2E_Counter], 1);
    put(p, in[VR_PLN_TimeStamp], 2);
    put(p, in[VR_PLN_AccelReq], 2);
    put(p, in[VR_PLN_SteerReq], 2);
    put(p, in[VR_PLN_SpeedReq], 2);
    put(p, in[VR_PLN_WdAnswer], 1);
    put(p, in[VR_PLN_PerceptionHealth], 1);
    put(p, in[VR_PLN_Flags], 1);
    int n = in[VR_PLN_Command_DLC];
    n = n < 0 ? 0 : (n > 14 ? 14 : n);
    if (x->n_pending < 16) {
      Frame &f = x->pending[x->n_pending++];
      f.id = CMD_ID;
      f.len = (uint8_t)n;
      memcpy(f.data, d, n);
    }
  }
  if (t % CYCLE_MS == 0) {
    const fmi2Real *r = x->reals;
    Feedback fb = {r[VR_VEH_Speed], r[VR_VEH_LongAccel], r[VR_VEH_RoadWheelAngle], r[VR_VEH_YawRate], r[VR_VEH_GradeAccel]};
    uint8_t act[8];
    if (sc.cycle(t, x->pending, x->n_pending, fb, act) == 7) {  // SAF_ActuatorCmd: <BBhhB
      in[VR_SAF_E2E_CRC] = act[0];
      in[VR_SAF_E2E_Counter] = act[1] & 0x0F;
      in[VR_SAF_AccelCmd] = (int16_t)(act[2] | act[3] << 8);
      in[VR_SAF_SteerCmd] = (int16_t)(act[4] | act[5] << 8);
      in[VR_SAF_BackupBrake] = act[6] & 1;
      in[VR_SAF_ActuatorCmd_TxCounter] += 1;
    }
    x->n_pending = 0;
  }
  publish(x);
}

}  // namespace

extern "C" {

const char *fmi2GetTypesPlatform() { return fmi2TypesPlatform; }
const char *fmi2GetVersion() { return fmi2Version; }

fmi2Status fmi2SetDebugLogging(fmi2Component, fmi2Boolean, size_t, const fmi2String[]) { return fmi2OK; }

fmi2Component fmi2Instantiate(fmi2String name, fmi2Type type, fmi2String guid, fmi2String, const fmi2CallbackFunctions *cb,
                              fmi2Boolean, fmi2Boolean) {
  if (type != fmi2CoSimulation || !guid || strcmp(guid, MODEL_GUID) != 0) return 0;
  Inst *x = new Inst();
  if (cb) x->cb = *cb;
  snprintf(x->name, sizeof x->name, "%s", name ? name : "SafeStopVecuC");
  set_defaults(x);
  return x;
}
void fmi2FreeInstance(fmi2Component c) { delete (Inst *)c; }

fmi2Status fmi2SetupExperiment(fmi2Component, fmi2Boolean, fmi2Real, fmi2Real, fmi2Boolean, fmi2Real) { return fmi2OK; }
fmi2Status fmi2EnterInitializationMode(fmi2Component c) {
  ((Inst *)c)->init_mode = true;
  return fmi2OK;
}
fmi2Status fmi2ExitInitializationMode(fmi2Component c) {
  Inst *x = (Inst *)c;
  x->init_mode = false;
  fmi2Status s = start(x);
  x->initialized = s == fmi2OK;
  return s;
}
fmi2Status fmi2Terminate(fmi2Component) { return fmi2OK; }
fmi2Status fmi2Reset(fmi2Component c) {
  set_defaults((Inst *)c);
  return fmi2OK;
}

fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t n, fmi2Real v[]) {
  Inst *x = (Inst *)c;
  for (size_t i = 0; i < n; i++) {
    if (vr[i] >= N_REAL) return fmi2Error;
    v[i] = x->reals[vr[i]];
  }
  return fmi2OK;
}
fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t n, fmi2Integer v[]) {
  Inst *x = (Inst *)c;
  for (size_t i = 0; i < n; i++) {
    if (vr[i] == VR_Bench_Defects) v[i] = x->defects;
    else if (vr[i] < N_INT) v[i] = x->ints[vr[i]];
    else return fmi2Error;
  }
  return fmi2OK;
}
fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t n, fmi2Boolean v[]) {
  Inst *x = (Inst *)c;
  for (size_t i = 0; i < n; i++) {
    if (vr[i] == VR_Bench_WarmStart) v[i] = x->warm_start;
    else if (vr[i] < N_BOOL) v[i] = x->bools[vr[i]];
    else return fmi2Error;
  }
  return fmi2OK;
}
fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference vr[], size_t n, fmi2String v[]) {
  Inst *x = (Inst *)c;
  for (size_t i = 0; i < n; i++) {
    if (vr[i] != VR_Bench_Config) return fmi2Error;
    v[i] = x->config;
  }
  return fmi2OK;
}

// Parameters are "fixed": settable only before initialisation ends (FMI 2.0 table, section 4.2.4).
fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t n, const fmi2Real v[]) {
  Inst *x = (Inst *)c;
  for (size_t i = 0; i < n; i++) {
    if (vr[i] > VR_VEH_GradeAccel) return fmi2Error;  // only the five VEH_* inputs are settable
    x->reals[vr[i]] = v[i];
  }
  return fmi2OK;
}
fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t n, const fmi2Integer v[]) {
  Inst *x = (Inst *)c;
  for (size_t i = 0; i < n; i++) {
    if (vr[i] == VR_Bench_Defects) {
      if (x->initialized) return fmi2Error;
      x->defects = v[i];
    } else if (vr[i] <= VR_PLN_WdKickCounter) {
      x->ints[vr[i]] = v[i];
    } else {
      log(x, fmi2Error, "fmi2SetInteger: not an input");
      return fmi2Error;
    }
  }
  return fmi2OK;
}
fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t n, const fmi2Boolean v[]) {
  Inst *x = (Inst *)c;
  for (size_t i = 0; i < n; i++) {
    if (vr[i] == VR_Bench_WarmStart) {
      if (x->initialized) return fmi2Error;
      x->warm_start = v[i];
    } else if (vr[i] < N_BOOL) {
      x->bools[vr[i]] = v[i];
    } else {
      return fmi2Error;
    }
  }
  return fmi2OK;
}
fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference vr[], size_t n, const fmi2String v[]) {
  Inst *x = (Inst *)c;
  for (size_t i = 0; i < n; i++) {
    if (vr[i] != VR_Bench_Config || x->initialized || !v[i]) return fmi2Error;
    snprintf(x->config, sizeof x->config, "%s", v[i]);
  }
  return fmi2OK;
}

fmi2Status fmi2DoStep(fmi2Component c, fmi2Real t, fmi2Real h, fmi2Boolean) {
  Inst *x = (Inst *)c;
  if (!x->initialized) return fmi2Error;
  // The controller is a 1 ms step machine (its own 10 ms task inside). Larger steps are split into 1 ms sub-steps.
  int64_t t0 = (int64_t)(t * 1000.0 + 0.5), n = (int64_t)(h * 1000.0 + 0.5);
  if (n < 1) n = 1;
  for (int64_t k = 0; k < n; k++) step(x, t0 + k);
  return fmi2OK;
}

// Not supported: state get/set, directional derivatives, asynchronous steps (capability flags say so).
fmi2Status fmi2GetFMUstate(fmi2Component, fmi2FMUstate *) { return fmi2Error; }
fmi2Status fmi2SetFMUstate(fmi2Component, fmi2FMUstate) { return fmi2Error; }
fmi2Status fmi2FreeFMUstate(fmi2Component, fmi2FMUstate *) { return fmi2Error; }
fmi2Status fmi2SerializedFMUstateSize(fmi2Component, fmi2FMUstate, size_t *) { return fmi2Error; }
fmi2Status fmi2SerializeFMUstate(fmi2Component, fmi2FMUstate, fmi2Byte[], size_t) { return fmi2Error; }
fmi2Status fmi2DeSerializeFMUstate(fmi2Component, const fmi2Byte[], size_t, fmi2FMUstate *) { return fmi2Error; }
fmi2Status fmi2GetDirectionalDerivative(fmi2Component, const fmi2ValueReference[], size_t, const fmi2ValueReference[], size_t,
                                        const fmi2Real[], fmi2Real[]) {
  return fmi2Error;
}
fmi2Status fmi2SetRealInputDerivatives(fmi2Component, const fmi2ValueReference[], size_t, const fmi2Integer[], const fmi2Real[]) {
  return fmi2Error;
}
fmi2Status fmi2GetRealOutputDerivatives(fmi2Component, const fmi2ValueReference[], size_t, const fmi2Integer[], fmi2Real[]) {
  return fmi2Error;
}
fmi2Status fmi2CancelStep(fmi2Component) { return fmi2Error; }
fmi2Status fmi2GetStatus(fmi2Component, const fmi2StatusKind, fmi2Status *) { return fmi2Discard; }
fmi2Status fmi2GetRealStatus(fmi2Component, const fmi2StatusKind, fmi2Real *) { return fmi2Discard; }
fmi2Status fmi2GetIntegerStatus(fmi2Component, const fmi2StatusKind, fmi2Integer *) { return fmi2Discard; }
fmi2Status fmi2GetBooleanStatus(fmi2Component, const fmi2StatusKind, fmi2Boolean *) { return fmi2Discard; }
fmi2Status fmi2GetStringStatus(fmi2Component, const fmi2StatusKind, fmi2String *) { return fmi2Discard; }

}  // extern "C"
