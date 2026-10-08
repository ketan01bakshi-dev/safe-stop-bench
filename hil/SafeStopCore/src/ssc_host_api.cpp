// C API for the PC build only (compiled with -DSSC_HOST_API by scripts/build_native.py; empty on the ESP32).
// Two levels: the controller alone (native SiL, NativeDUT) and the whole node with its link protocol (LoopbackDUT:
// the exact firmware logic of board B, minus the Arduino shim, in lockstep with the bench).
#ifdef SSC_HOST_API
#include <string.h>

#include <vector>

#include "ssc_core.h"

#if defined(_WIN32)
#define SSC_EXPORT extern "C" __declspec(dllexport)
#else
#define SSC_EXPORT extern "C" __attribute__((visibility("default")))
#endif

using namespace ssc;

struct Ctrl {
  Controller c;
  Frame q[16];
  int n;
};

SSC_EXPORT void *ssc_ctrl_new() { return new Ctrl(); }
SSC_EXPORT void ssc_ctrl_free(void *h) { delete (Ctrl *)h; }
SSC_EXPORT void ssc_ctrl_init(void *h, const double *cfg, uint32_t defects, int warm) {
  Config c;
  config_from_array(c, cfg);
  ((Ctrl *)h)->c.init(c, defects, warm != 0);
  ((Ctrl *)h)->n = 0;
}
SSC_EXPORT void ssc_ctrl_kick(void *h, int64_t t) { ((Ctrl *)h)->c.kick(t); }
SSC_EXPORT int ssc_ctrl_release(void *h, int64_t t, double v) { return ((Ctrl *)h)->c.release(t, v); }
SSC_EXPORT void ssc_ctrl_brownout(void *h, int64_t t, int on) { ((Ctrl *)h)->c.brownout(t, on != 0); }
SSC_EXPORT void ssc_ctrl_set_tx_ok(void *h, int ok) { ((Ctrl *)h)->c.tx_ok = ok != 0; }
SSC_EXPORT void ssc_ctrl_push(void *h, int id, const uint8_t *d, int n) {
  Ctrl *x = (Ctrl *)h;
  if (x->n >= 16 || n > 16) return;
  x->q[x->n].id = (uint16_t)id;
  x->q[x->n].len = (uint8_t)n;
  memcpy(x->q[x->n].data, d, n);
  x->n++;
}
SSC_EXPORT int ssc_ctrl_cycle(void *h, int64_t t, double v, double a, double delta, double yaw, double grade, uint8_t *act) {
  Ctrl *x = (Ctrl *)h;
  Feedback fb = {v, a, delta, yaw, grade};
  int k = x->c.cycle(t, x->q, x->n, fb, act);
  x->n = 0;
  return k;
}
// ints: state (OFF if unpowered), cause, challenge, mrm, backup, rejected, release_rejected, has_fault; reals: out_a, out_s, t_fault
SSC_EXPORT void ssc_ctrl_get(void *h, int *ints, double *reals) {
  const Controller &c = ((Ctrl *)h)->c;
  ints[0] = c.powered ? c.state : OFF;
  ints[1] = c.cause;
  ints[2] = c.challenge;
  ints[3] = c.mrm_pull_over;
  ints[4] = c.out_backup;
  ints[5] = (int)c.rejected;
  ints[6] = (int)c.release_rejected;
  ints[7] = c.has_fault;
  reals[0] = c.out_a;
  reals[1] = c.out_s;
  reals[2] = (double)c.t_fault;
}

// ---- node level ------------------------------------------------------------------------------------------------------------
struct HostNode {
  Node node;
  std::vector<uint8_t> out;
};
static void host_ser_tx(void *ctx, const uint8_t *d, size_t n) {
  std::vector<uint8_t> &o = ((HostNode *)ctx)->out;
  o.insert(o.end(), d, d + n);
}
SSC_EXPORT void *ssc_node_new() {
  HostNode *h = new HostNode();
  NodeIo io = {0, host_ser_tx, 0, h};  // no CAN, no timer: the loopback carries everything on the "serial" link
  h->node.begin(io, "SafeStopNode host-loopback");
  return h;
}
SSC_EXPORT void ssc_node_free(void *h) { delete (HostNode *)h; }
SSC_EXPORT void ssc_node_feed(void *h, const uint8_t *d, int n, int64_t now) { ((HostNode *)h)->node.feed(d, (size_t)n, now); }
SSC_EXPORT void ssc_node_poll(void *h, int64_t now) { ((HostNode *)h)->node.poll(now); }
SSC_EXPORT int ssc_node_read(void *h, uint8_t *buf, int cap) {
  std::vector<uint8_t> &o = ((HostNode *)h)->out;
  int n = (int)o.size() < cap ? (int)o.size() : cap;
  memcpy(buf, o.data(), n);
  o.erase(o.begin(), o.begin() + n);
  return n;
}
// v2.12: the stored configuration and the boot path, so the board emulator can model a power cut (the firmware keeps
// the blob in NVS and calls restore() at boot)
SSC_EXPORT int ssc_node_config(void *h, uint8_t *buf, int cap) {
  uint8_t n = 0;
  const uint8_t *b = ((HostNode *)h)->node.config_blob(&n);
  int k = (int)n < cap ? (int)n : cap;
  memcpy(buf, b, k);
  return k;
}
SSC_EXPORT int ssc_node_restore(void *h, const uint8_t *blob, int n, int64_t now) {
  return ((HostNode *)h)->node.restore(blob, (uint8_t)n, now) ? 1 : 0;
}
SSC_EXPORT int ssc_n_config() { return N_CONFIG; }
#endif
