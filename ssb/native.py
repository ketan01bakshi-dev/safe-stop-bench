"""The ESP32 firmware core (hil/SafeStopCore, C++) running on the PC, driven through ctypes.

Two adapters, both deterministic (the bench owns time):
- NativeDUT: the C++ controller alone, stepped exactly like ReferenceDUT. Proves the port: an exact back-to-back
  against the Python reference must show no difference at all.
- LoopbackLink: the whole node of board B (controller + link protocol + scheduling), fed bytes in lockstep. With
  ssb.hil.LinkDUT on top, this is the firmware's logic minus only the Arduino shim (Serial, MCP2515, millis).

Build the DLL first: python scripts/build_native.py (needs: pip install ziglang).
"""
from __future__ import annotations

import ctypes
import hashlib
import sys
from pathlib import Path

from .dut import BlackBoxObserver, DeviceUnderTest, Outputs
from .safety import CYCLE_MS, MUTANTS

ROOT = Path(__file__).resolve().parent.parent
DLL = ROOT / "hil" / "build" / ("ssc_native.dll" if sys.platform == "win32" else "libssc_native.so")
CORE_SRC = ROOT / "hil" / "SafeStopCore" / "src" / "ssc_core.cpp"
# Order of ssc::Config (hil/SafeStopCore/src/ssc_core.h). The calibration is downloaded at every reset.
CFG_KEYS = ["cmd_timeout_ms", "max_age_ms", "wd_window_min_ms", "wd_window_max_ms", "wd_early_kicks", "steer_rate_max_dps",
            "steer_rate_slope", "steer_rate_min_dps", "odd_max_kmh", "max_accel", "min_accel", "max_jerk", "env_debounce_ms",
            "steer_mismatch_deg", "steer_mismatch_ms", "brake_min_ratio", "brake_mismatch_ms", "degraded_cap_kmh",
            "degraded_recover_ms", "mrm_decel", "mrm_jerk", "brake_only_decel", "emergency_jerk", "release_clear_ms"]
STATES = ["INIT", "NORMAL", "DEGRADED", "PULL_OVER", "STOP_IN_LANE", "BRAKE_ONLY_STOP", "BACKUP_BRAKE_STOP", "OFF"]

_lib = None


def lib():
    global _lib
    if _lib is None:
        if not DLL.exists():
            raise RuntimeError(f"{DLL} not built: run scripts/build_native.py (needs: pip install ziglang)")
        L = ctypes.CDLL(str(DLL))
        P, I64, D, U8P, I = ctypes.c_void_p, ctypes.c_int64, ctypes.c_double, ctypes.POINTER(ctypes.c_uint8), ctypes.c_int
        sig = {"ssc_ctrl_new": ([], P), "ssc_ctrl_free": ([P], None),
               "ssc_ctrl_init": ([P, ctypes.POINTER(D), ctypes.c_uint32, I], None), "ssc_ctrl_kick": ([P, I64], None),
               "ssc_ctrl_release": ([P, I64, D], I), "ssc_ctrl_brownout": ([P, I64, I], None),
               "ssc_ctrl_set_tx_ok": ([P, I], None), "ssc_ctrl_push": ([P, I, ctypes.c_char_p, I], None),
               "ssc_ctrl_cycle": ([P, I64, D, D, D, D, D, U8P], I),
               "ssc_ctrl_get": ([P, ctypes.POINTER(I), ctypes.POINTER(D)], None),
               "ssc_node_new": ([], P), "ssc_node_free": ([P], None), "ssc_node_feed": ([P, ctypes.c_char_p, I, I64], None),
               "ssc_node_poll": ([P, I64], None), "ssc_node_read": ([P, U8P, I], I), "ssc_n_config": ([], I)}
        for name, (args, res) in sig.items():
            f = getattr(L, name)
            f.argtypes, f.restype = args, res
        if L.ssc_n_config() != len(CFG_KEYS):
            raise RuntimeError("ssc_core.h Config and ssb.native.CFG_KEYS disagree; rebuild the DLL")
        _lib = L
    return _lib


def cfg_array(cfg: dict) -> list[float]:
    return [float(cfg["safety"][k]) for k in CFG_KEYS]


def defects_mask(defects) -> int:
    return sum(1 << k for k, n in enumerate(MUTANTS) if n in defects)


def core_identity() -> str:
    return f"SafeStopCore sha256:{hashlib.sha256(CORE_SRC.read_bytes()).hexdigest()[:12]}"


class NativeDUT(BlackBoxObserver, DeviceUnderTest):
    """The C++ controller, stepped like ReferenceDUT: 1 ms steps, its own 10 ms cycle."""

    def __init__(self, cfg: dict, defects=frozenset(), warm_start: bool = True):
        self.L, self.cfg, self.defects = lib(), cfg, frozenset(defects)
        self.h = self.L.ssc_ctrl_new()
        self.cfgv = (ctypes.c_double * len(CFG_KEYS))(*cfg_array(cfg))
        self.name = "native C++ core" + (f"[{','.join(sorted(self.defects))}]" if self.defects else "")
        self.ints, self.reals = (ctypes.c_int * 8)(), (ctypes.c_double * 3)()
        self.act = (ctypes.c_uint8 * 8)()
        self.prepare(warm_start, self.defects)

    def prepare(self, warm_start: bool, defects=frozenset()) -> None:
        self.defects = frozenset(defects)
        self.L.ssc_ctrl_init(self.h, self.cfgv, defects_mask(self.defects), int(warm_start))
        self.last = Outputs(state="NORMAL" if warm_start else "INIT")
        self._observe_reset()

    def step(self, t, frames, kicks, fb, release, power_ok, tx_ok) -> Outputs:
        L, h = self.L, self.h
        L.ssc_ctrl_brownout(h, t, int(not power_ok))
        L.ssc_ctrl_set_tx_ok(h, int(tx_ok))
        for k in kicks:
            L.ssc_ctrl_kick(h, k)
        if release:
            L.ssc_ctrl_release(h, t, fb["v"])
        for f in frames:
            L.ssc_ctrl_push(h, f.can_id, bytes(f.data), len(f.data))
        act = []
        if t % CYCLE_MS == 0:
            n = L.ssc_ctrl_cycle(h, t, fb["v"], fb["a"], fb["delta"], fb["yaw_rate"], fb.get("grade_accel", 0.0), self.act)
            if n:
                act = [(0x200, bytes(self.act[:n]))]
        L.ssc_ctrl_get(h, self.ints, self.reals)
        from .canio import CAUSES
        st, cause = STATES[self.ints[0]], CAUSES[self.ints[1]]
        self._observe(t, self.last.state, st, cause, release)
        self.last = Outputs(act, st, cause, self.ints[2], "PULL_OVER" if self.ints[3] else None,
                            (self.reals[0], self.reals[1], bool(self.ints[4])))
        return self.last

    def identity(self) -> str:
        return f"{self.name} {core_identity()}"

    def close(self) -> None:
        if self.h:
            self.L.ssc_ctrl_free(self.h)
            self.h = None


class LoopbackLink:
    """Board B's node logic on the PC, behind the same byte-stream interface as a serial port."""
    realtime = False

    def __init__(self):
        self.L = lib()
        self.h = self.L.ssc_node_new()
        self.buf = (ctypes.c_uint8 * 65536)()
        self.name = "loopback (host build of board B's node logic)"

    def write(self, data: bytes, now_ms: int) -> None:
        self.L.ssc_node_feed(self.h, data, len(data), now_ms)

    def poll(self, now_ms: int) -> None:
        self.L.ssc_node_poll(self.h, now_ms)

    def read(self) -> bytes:
        n = self.L.ssc_node_read(self.h, self.buf, len(self.buf))
        return bytes(self.buf[:n])

    def close(self) -> None:
        if self.h:
            self.L.ssc_node_free(self.h)
            self.h = None
