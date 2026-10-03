"""SafeStopVecu: the reference safety controller packaged as an FMI 2.0 co-simulation FMU (PythonFMU).

This is the stand-in for a supplier's vECU FMU. It is built by scripts/build_fmu.py, which stages the controller code
as the package `vecu_core` inside the FMU's resources/ folder, so the FMU never imports the bench's own `ssb` package:
the bench sees a zip with a modelDescription.xml and a binaries/win64 DLL, exactly as with a supplier FMU.

Interface (see ssb/fmu_contract.py): raw DBC signals in and out, Rx/Tx counters as frame events, a 1 ms
communication step; the controller runs its own 10 ms task inside. Honest limit: a PythonFMU DLL embeds the
host's Python, whereas a supplier FMU is compiled C. The bench side (FMPy -> FMI 2.0 C API -> DLL) is the same.
"""
import json
import os
import struct
import sys

from pythonfmu import Boolean, Fmi2Causality, Fmi2Initial, Fmi2Slave, Fmi2Variability, Integer, Real, String

PLN = ["PLN_E2E_CRC", "PLN_E2E_Counter", "PLN_TimeStamp", "PLN_AccelReq", "PLN_SteerReq", "PLN_SpeedReq",
       "PLN_WdAnswer", "PLN_PerceptionHealth", "PLN_Flags"]
PLN_FMT = "<HBHhhHBBB"   # 14 bytes, same layout as PLN_Command in dbc/safe_stop.dbc
ACT_FMT = "<BBhhB"       # 7 bytes, SAF_ActuatorCmd
VEH = ["VEH_Speed", "VEH_LongAccel", "VEH_RoadWheelAngle", "VEH_YawRate", "VEH_GradeAccel"]
STATES = ["INIT", "NORMAL", "DEGRADED", "PULL_OVER", "STOP_IN_LANE", "BRAKE_ONLY_STOP", "BACKUP_BRAKE_STOP", "OFF"]
CAUSES = [None, "TIMEOUT", "E2E_INVALID", "STALE_DATA", "WATCHDOG_LATE", "WATCHDOG_EARLY", "WATCHDOG_QA", "ENVELOPE",
          "STEER_ACTUATOR", "BRAKE_ACTUATOR", "ACT_BUS_OFF", "PERCEPTION_DEGRADED", "ODD_EXIT", "PERCEPTION_LOST", "SAFETY_RESET"]


class _Frame:
    def __init__(self, can_id, data):
        self.can_id, self.data = can_id, data


class SafeStopVecu(Fmi2Slave):
    author = "Ketan Bakshi (safe-stop bench v2)"
    description = "Reference safety controller (checker) for a planner -> actuator chain; illustrative, synthetic limits"

    # Exported names, units and state codes. The reference FMU uses the bench names; a subclass can rename (see
    # SupplierStyleVecu.py), which is how the bench's mapping file is tested against an FMU it doesn't "know".
    NAMES: dict = {}           # bench name -> exported name; None = not exported
    SPEED_UNIT = 1.0           # exported VEH_Speed = m/s * SPEED_UNIT (3.6 = km/h)
    STATE_CODES = STATES       # exported SAF_State value = index in this list

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.Bench_WarmStart, self.Bench_Defects, self.Bench_Config = True, 0, "default"
        self.PLN_Command_RxCounter, self.PLN_Command_DLC, self.PLN_WdKickCounter = 0, 14, 0
        for n in PLN:
            setattr(self, n, 0)
        for n in VEH:
            setattr(self, n, 0.0)
        self.Bench_PowerOk, self.Bench_TxOk, self.Bench_Release = True, True, False
        for n in ("SAF_ActuatorCmd_TxCounter", "SAF_E2E_CRC", "SAF_E2E_Counter", "SAF_AccelCmd", "SAF_SteerCmd",
                  "SAF_BackupBrake", "SAF_State", "SAF_Cause", "SAF_WdChallenge", "SAF_MrmRequest"):
            setattr(self, n, 0)
        self.SAF_AccelOut, self.SAF_SteerOut = 0.0, 0.0
        self.sc = None

        P, I, O = Fmi2Causality.parameter, Fmi2Causality.input, Fmi2Causality.output
        fixed, discrete = Fmi2Variability.fixed, Fmi2Variability.discrete   # FMI 2.0: Integer/Boolean can't be continuous
        self._reg(Boolean, "Bench_WarmStart", P, fixed)
        self._reg(Integer, "Bench_Defects", P, fixed)
        self._reg(String, "Bench_Config", P, fixed)
        for n in ["PLN_Command_RxCounter", "PLN_Command_DLC"] + PLN + ["PLN_WdKickCounter"]:
            self._reg(Integer, n, I, discrete)
        for n in VEH:
            self._reg(Real, n, I, None, scale=self.SPEED_UNIT if n == "VEH_Speed" else 1.0)
        for n in ("Bench_PowerOk", "Bench_TxOk", "Bench_Release"):
            self._reg(Boolean, n, I, discrete)
        for n in ("SAF_ActuatorCmd_TxCounter", "SAF_E2E_CRC", "SAF_E2E_Counter", "SAF_AccelCmd", "SAF_SteerCmd",
                  "SAF_BackupBrake", "SAF_State", "SAF_Cause", "SAF_WdChallenge", "SAF_MrmRequest"):
            self._reg(Integer, n, O, discrete)
        self._reg(Real, "SAF_AccelOut", O, None)
        self._reg(Real, "SAF_SteerOut", O, None)

    def _reg(self, kind, bench, causality, variability, scale=1.0):
        name = self.NAMES.get(bench, bench)
        if name is None:
            return
        kw = dict(causality=causality, getter=lambda: getattr(self, bench) * scale if scale != 1.0 else getattr(self, bench))
        if variability is not None:
            kw["variability"] = variability
        if causality == Fmi2Causality.output:   # "exact" + start value: valid without InitialUnknowns (FMI 2.0 rule)
            kw["initial"] = Fmi2Initial.exact
        else:
            kw["setter"] = lambda v: setattr(self, bench, v / scale if scale != 1.0 else v)
        self.register_variable(kind(name, **kw))

    def exit_initialization_mode(self):
        if self.resources and self.resources not in sys.path:
            sys.path.insert(0, self.resources)
        from vecu_core.safety import MUTANTS, SafetyController   # staged copy, never the bench's `ssb`
        with open(os.path.join(self.resources, "configs", f"{self.Bench_Config}.json"), encoding="utf-8") as f:
            cfg = json.load(f)
        names = list(MUTANTS)
        defects = frozenset(n for k, n in enumerate(names) if int(self.Bench_Defects) >> k & 1)
        self.sc = SafetyController(cfg, defects, warm_start=bool(self.Bench_WarmStart))
        self.cycle_ms = 10
        self.pending, self.rx_seen, self.kick_seen = [], self.PLN_Command_RxCounter, self.PLN_WdKickCounter
        self._publish()

    def do_step(self, current_time, step_size):
        t = int(round(current_time * 1000))
        sc = self.sc
        sc.brownout(t, not self.Bench_PowerOk)
        sc.tx_ok = bool(self.Bench_TxOk)
        for _ in range((self.PLN_WdKickCounter - self.kick_seen) & 0xFF):
            sc.kick(t)
        self.kick_seen = self.PLN_WdKickCounter
        if self.Bench_Release:
            sc.release(t, self.VEH_Speed)
        if self.PLN_Command_RxCounter != self.rx_seen:
            self.rx_seen = self.PLN_Command_RxCounter
            data = struct.pack(PLN_FMT, *[int(getattr(self, n)) for n in PLN])
            self.pending.append(_Frame(0x100, data[:max(0, int(self.PLN_Command_DLC))]))
        if t % self.cycle_ms == 0:
            fb = {"v": self.VEH_Speed, "a": self.VEH_LongAccel, "delta": self.VEH_RoadWheelAngle,
                  "yaw_rate": self.VEH_YawRate, "grade_accel": self.VEH_GradeAccel}
            for _, data in sc.cycle(t, self.pending, fb):
                crc, ctr, a, s, b = struct.unpack(ACT_FMT, data)
                self.SAF_E2E_CRC, self.SAF_E2E_Counter, self.SAF_AccelCmd, self.SAF_SteerCmd, self.SAF_BackupBrake = crc, ctr & 0x0F, a, s, b & 1
                self.SAF_ActuatorCmd_TxCounter += 1
            self.pending = []
        self._publish()
        return True

    def _publish(self):
        sc = self.sc
        self.SAF_State = self.STATE_CODES.index(sc.state if sc.powered else "OFF")
        self.SAF_Cause = CAUSES.index(sc.cause) if sc.cause in CAUSES else 0
        self.SAF_WdChallenge = int(sc.challenge)
        self.SAF_MrmRequest = 1 if sc.mrm_request == "PULL_OVER" else 0
        self.SAF_AccelOut, self.SAF_SteerOut = float(sc.out[0]), float(sc.out[1])
