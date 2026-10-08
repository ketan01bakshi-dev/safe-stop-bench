"""Device-under-test (DUT) interface and adapters.

The runner only talks to `DeviceUnderTest`. Today's reference controller is one adapter; a supplied
virtual ECU (FMU, CAN process, ROS 2 node, shared library) is another. See docs/HOW_IT_WORKS_AND_TESTING_A_SUPPLIED_VECU.md.

Status (v2.2): all three are tested. ReferenceDUT in-process; CanDUT = a separate process over python-can (v2.1);
FmuDUT = an FMI 2.0 co-simulation FMU via FMPy (v2.2). CanDUT and FmuDUT reproduce the reference matrix result.
"""
from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from pathlib import Path

from . import e2e
from .safety import CYCLE_MS, SafetyController


@dataclass
class Outputs:
    act_frames: list = field(default_factory=list)   # (can_id, bytes) for the actuator bus
    state: str = "NORMAL"
    cause: str | None = None
    challenge: int = 0
    mrm_request: str | None = None
    out_cmd: tuple = (0.0, 0.0, False)               # (accel, steer, backup) the DUT is commanding
    # (DUT cycle time in ms, commanded steer) from the DUT's own clock, when the bench only sees it through a link
    # (HiL/PiL). The slew invariant then uses this time base, not the PC's arrival time (v2.9.1).
    cycle: tuple | None = None


class BenchFault(RuntimeError):
    """The bench (observer, link, bus node) failed, not the DUT. The runner reruns the scenario instead of judging it."""


class DeviceUnderTest:
    name = "abstract"
    defects: frozenset = frozenset()   # seeded mutants (reference-derived DUTs only)

    def reset(self) -> None: ...
    def step(self, t: int, frames: list, kicks: list[int], fb: dict, release: bool, power_ok: bool, tx_ok: bool) -> Outputs:
        raise NotImplementedError
    def identity(self) -> str:
        return self.name


class ReferenceDUT(DeviceUnderTest):
    """Our reference safety controller, stepped every 1 ms; it runs its own 10 ms cycle."""

    def __init__(self, cfg: dict, defects: frozenset = frozenset(), warm_start: bool = True):
        self.cfg, self.defects, self.warm = cfg, frozenset(defects), warm_start
        self.name = "reference" + (f"[{','.join(sorted(defects))}]" if defects else "")
        self.reset()

    def reset(self) -> None:
        self.sc = SafetyController(self.cfg, self.defects, self.warm)
        self.pending: list = []
        self.last = Outputs()

    def step(self, t, frames, kicks, fb, release, power_ok, tx_ok) -> Outputs:
        sc = self.sc
        sc.brownout(t, not power_ok)
        sc.tx_ok = tx_ok
        for k in kicks:
            sc.kick(k)
        if release:
            sc.release(t, fb["v"])
        self.pending.extend(frames)
        act = []
        if t % CYCLE_MS == 0:
            act = sc.cycle(t, self.pending, fb)
            self.pending = []
        self.last = Outputs(act, sc.state if sc.powered else "OFF", sc.cause, sc.challenge, sc.mrm_request, sc.out)
        return self.last

    def identity(self) -> str:
        src = (Path(__file__).parent / "safety.py").read_bytes()
        return f"{self.name} sha256:{hashlib.sha256(src).hexdigest()[:12]}"


class BlackBoxObserver:
    """What the oracle needs, derived from a black box's OUTPUTS only: first reaction time and cause, the state
    events, and whether an operator release was accepted. Used by every adapter that can't see inside the DUT."""

    def _observe_reset(self) -> None:
        from types import SimpleNamespace
        self.sc = SimpleNamespace(t_fault=None, cause=None, events=[], dtcs=[], release_rejected=0, rejected=0)
        self.release_check: int | None = None

    def _observe(self, t: int, prev: str, st: str, cause, release: bool) -> None:
        from .safety import RANK
        obs = self.sc
        if st != prev:
            obs.events.append((t, f"{prev} → {st} ({cause})"))
            if obs.t_fault is None and RANK.get(st, 0) >= 1:   # first reaction seen on the outputs
                obs.t_fault, obs.cause = t, cause
        if release:
            self.release_check = t + 60
        if self.release_check is not None and t >= self.release_check:
            accepted = st == "INIT" or any("→ INIT" in txt for tt, txt in obs.events if tt >= self.release_check - 60)
            obs.events.append((t, "release accepted → INIT" if accepted else "release rejected"))
            obs.release_rejected += 0 if accepted else 1
            self.release_check = None


class FmuDUT(BlackBoxObserver, DeviceUnderTest):
    """A co-simulation FMU (FMI 2.0) loaded in-process with FMPy and stepped every 1 ms of simulated time.

    `mapping` (JSON, see ssb/fmu_contract.py and fmu/mapping_reference.json) maps the bench's names to the FMU's:
    {"inputs": {bench: fmu}, "outputs": {bench: fmu}, "parameters": {bench: fmu}, "state_values": {"0": "INIT", ...}}.
    A Real entry may be {"name": fmu, "factor": f, "offset": o} (fmu = bench * f + o) when units differ.
    Frames cross as raw DBC signals plus an Rx/Tx counter per message (FMI 2.0 has no events). If two planner frames
    arrive in the same millisecond, the second is delivered 1 ms later (a one-deep receive queue); the controller
    runs a 10 ms task, so this only matters when a task boundary falls exactly between them.
    Not real time: the FMU runs as fast as it can, deterministically (same seed, same result).
    """

    def __init__(self, fmu_path: str, mapping: dict | None = None, config_name: str = "default"):
        try:
            from fmpy import extract, read_model_description
            from fmpy.fmi2 import FMU2Slave
        except ImportError as e:  # pragma: no cover
            raise RuntimeError("FmuDUT needs FMPy: pip install fmpy") from e
        from . import canio, fmu_contract
        self.canio, self.contract = canio, fmu_contract
        self.path, self.config_name = str(fmu_path), config_name
        self.md = md = read_model_description(self.path, validate=True)
        if md.coSimulation is None:
            raise ValueError(f"{Path(fmu_path).name}: not a co-simulation FMU (model exchange only) - needs a solver wrapper")
        if not md.fmiVersion.startswith("2."):
            raise ValueError(f"{Path(fmu_path).name}: FMI {md.fmiVersion}; this adapter drives FMI 2.0")
        self.map = mapping or fmu_contract.identity_mapping()
        self.vars = {v.name: v for v in md.modelVariables}
        self._resolve()
        self.unzip = extract(self.path)
        self.fmu = FMU2Slave(guid=md.guid, unzipDirectory=self.unzip, modelIdentifier=md.coSimulation.modelIdentifier,
                             instanceName="dut")
        self.db = canio.load_dbc()
        self.name = f"fmu:{Path(fmu_path).name}"
        self.defects: frozenset = frozenset()
        self.live = False
        self.prepare(warm_start=True)

    # ---- mapping -------------------------------------------------------------------------------------------------
    def _resolve(self) -> None:
        """Check the mapping against the FMU: every required bench name mapped, to a variable that exists, with the
        right causality and type. Collect every problem, then fail once with the full list."""
        problems, self.io = [], {}
        for kind, rows, section in (("input", self.contract.INPUTS, "inputs"), ("output", self.contract.OUTPUTS, "outputs"),
                                    ("parameter", self.contract.PARAMETERS, "parameters")):
            sec = self.map.get(section, {})
            for bench, typ, required, _ in rows:
                entry = sec.get(bench)
                if not entry:
                    if required:
                        problems.append(f"{kind} {bench}: not mapped (required)")
                    continue
                name = entry["name"] if isinstance(entry, dict) else entry
                v = self.vars.get(name)
                if v is None:
                    problems.append(f"{kind} {bench}: FMU has no variable '{name}'")
                    continue
                vt = "Integer" if v.type == "Enumeration" else v.type
                if v.causality != kind:
                    problems.append(f"{kind} {bench}: '{name}' has causality {v.causality}")
                if vt != typ:
                    problems.append(f"{kind} {bench}: '{name}' is {v.type}, bench expects {typ}")
                f, o = (entry.get("factor", 1.0), entry.get("offset", 0.0)) if isinstance(entry, dict) else (1.0, 0.0)
                self.io[bench] = (v.valueReference, vt, f, o)
        if problems:
            raise ValueError("FMU mapping problems:\n  " + "\n  ".join(problems))
        sv = self.map.get("state_values") or {str(k): v for k, v in self.contract.STATE_VALUES.items()}
        self.states = {int(k): v for k, v in sv.items()}
        self.out_names = [n for n, *_ in self.contract.OUTPUTS if n in self.io]

    def _set(self, values: dict) -> None:
        by: dict[str, tuple[list, list]] = {"Real": ([], []), "Integer": ([], []), "Boolean": ([], []), "String": ([], [])}
        for bench, val in values.items():
            vr, vt, f, o = self.io[bench]
            by[vt][0].append(vr)
            by[vt][1].append(val * f + o if vt == "Real" else (int(val) if vt == "Integer" else val))
        for vt, setter in (("Real", self.fmu.setReal), ("Integer", self.fmu.setInteger), ("Boolean", self.fmu.setBoolean),
                           ("String", self.fmu.setString)):
            if by[vt][0]:
                setter(by[vt][0], by[vt][1])

    def _get(self) -> dict:
        out = {}
        for vt, getter in (("Real", self.fmu.getReal), ("Integer", self.fmu.getInteger), ("Boolean", self.fmu.getBoolean)):
            names = [n for n in self.out_names if self.io[n][1] == vt]
            if names:
                for n, val in zip(names, getter([self.io[n][0] for n in names]), strict=True):
                    _, _, f, o = self.io[n]
                    out[n] = (val - o) / f if vt == "Real" else val
        return out

    # ---- lifecycle -----------------------------------------------------------------------------------------------
    def prepare(self, warm_start: bool, defects: frozenset = frozenset()) -> None:
        """Back to a clean start before every scenario: one instance per campaign, fmi2Reset between scenarios.
        One instance + fmi2Reset is cheaper than free/instantiate and needs no DLL reload (FMPy's freeInstance also
        unloads the DLL). `python -m ssb.fmu_inspect --lifecycle` tests both paths in a subprocess on intake."""
        self.defects = frozenset(defects)
        if self.live:
            self.fmu.terminate()
            self.fmu.reset()
        else:
            self.fmu.instantiate()
            self.live = True
        self.fmu.setupExperiment(startTime=0.0)
        params: dict[str, object] = {}
        if "Bench_WarmStart" in self.io:
            params["Bench_WarmStart"] = bool(warm_start)
        if "Bench_Config" in self.io:
            params["Bench_Config"] = self.config_name
        if self.defects:
            if "Bench_Defects" not in self.io:
                raise ValueError("--defect needs an FMU with a Bench_Defects parameter (the reference FMU)")
            from .safety import MUTANTS
            params["Bench_Defects"] = sum(1 << k for k, n in enumerate(MUTANTS) if n in self.defects)
        self._set(params)
        self.fmu.enterInitializationMode()
        self.fmu.exitInitializationMode()
        self.rx_n = self.kick_n = 0
        self.rx_queue: list = []
        self.cmd_signals = {n: 0 for n in self.contract.PLN_SIGNALS}
        self.dlc = 14
        o = self._get()
        self.tx_seen = o["SAF_ActuatorCmd_TxCounter"]
        self.last = Outputs(state=self.states.get(int(o["SAF_State"]), "NORMAL"))
        self._observe_reset()
        self.tx_merged = 0

    def step(self, t, frames, kicks, fb, release, power_ok, tx_ok) -> Outputs:
        cid = self.canio.ID
        self.rx_queue.extend(f for f in frames if f.can_id == cid["cmd"])
        if self.rx_queue:
            data = bytes(self.rx_queue.pop(0).data)
            self.cmd_signals = self.db.decode_message(cid["cmd"], data[:16].ljust(16, b"\0"), scaling=False,
                                                      decode_choices=False)
            self.dlc, self.rx_n = len(data), self.rx_n + 1
        self.kick_n = (self.kick_n + len(kicks)) & 0xFF
        vals = {n: self.cmd_signals[n] for n in self.contract.PLN_SIGNALS}
        vals.update(PLN_Command_RxCounter=self.rx_n, PLN_Command_DLC=self.dlc, PLN_WdKickCounter=self.kick_n,
                    VEH_Speed=fb["v"], VEH_LongAccel=fb["a"], VEH_RoadWheelAngle=fb["delta"], VEH_YawRate=fb["yaw_rate"],
                    VEH_GradeAccel=fb.get("grade_accel", 0.0), Bench_PowerOk=bool(power_ok), Bench_TxOk=bool(tx_ok),
                    Bench_Release=bool(release))
        self._set({n: v for n, v in vals.items() if n in self.io})
        self.fmu.doStep(currentCommunicationPoint=t / 1000, communicationStepSize=0.001)
        o = self._get()

        act = []
        tx = o["SAF_ActuatorCmd_TxCounter"]
        if tx != self.tx_seen:
            self.tx_merged += max(0, int(tx - self.tx_seen) - 1)   # >1 frame in one step: only the last is visible
            self.tx_seen = tx
            data = self.db.encode_message(cid["act"], {n: int(o[n]) for n in self.contract.SAF_SIGNALS}, scaling=False)
            act.append((cid["act"], bytes(data)))
        st = self.states.get(int(o["SAF_State"]), f"UNKNOWN_{int(o['SAF_State'])}")
        ci = int(o.get("SAF_Cause", 0))
        cause = self.canio.CAUSES[ci] if 0 <= ci < len(self.canio.CAUSES) else f"CAUSE_{ci}"
        if "SAF_AccelOut" in o and "SAF_SteerOut" in o:
            cmd = (o["SAF_AccelOut"], o["SAF_SteerOut"], bool(o["SAF_BackupBrake"]))
        else:
            cmd = (o["SAF_AccelCmd"] / 100, o["SAF_SteerCmd"] / 100, bool(o["SAF_BackupBrake"]))
        mrm = "PULL_OVER" if o.get("SAF_MrmRequest") else None
        self._observe(t, self.last.state, st, cause, release)
        self.last = Outputs(act, st, cause, int(o["SAF_WdChallenge"]), mrm, cmd)
        return self.last

    def close(self) -> None:
        if self.live:
            self.fmu.terminate()
            self.fmu.freeInstance()
            self.live = False

    def identity(self) -> str:
        md = self.md
        return (f"{self.name} sha256:{hashlib.sha256(Path(self.path).read_bytes()).hexdigest()[:12]} "
                f"(FMI {md.fmiVersion}, {md.generationTool}, guid {md.guid[:8]})")


class CanDUT(BlackBoxObserver, DeviceUnderTest):
    """A vECU in a SEPARATE PROCESS, reachable only over CAN (python-can + cantools DBC).

    By default it launches our reference controller as that process (`python -m ssb.vecu_process`), which makes it a
    true black box and lets back-to-back runs prove the adapter. For a supplied vECU, pass `launch=None` and start theirs
    on the same interface/channel; it must speak the DBC in dbc/safe_stop.dbc.
    Real time: the runner paces 1 simulated ms per wall-clock ms; timing now includes real transport and scheduling.

    v2.10: SAF_Status is E2E-protected (CRC-8 + alive counter, the C++ core's layout) and every status frame passes
    e2e.StatusReceiver, the same check as the HiL bench, before the observer uses it. `status_replay` puts a stale copy
    on the bus to prove it: {"capture_ms": t, "inject_ms": [t1, ...]} records the status frame seen at t and sends it
    again, byte for byte, from a second sender at each t_i. `status_e2e=False` turns the check off (the negative test).

    v2.11, `lockstep=True`: not paced. The bench sends every input for ms t and then VEH_Feedback(t) (always in that
    order), and at each 10 ms boundary waits until the vECU's status for that cycle has arrived; the SAF_Status alive
    counter, +1 per cycle since the reset, says which cycle a status belongs to. The vECU (told by BENCH_Lockstep) runs
    cycle T as soon as VEH_Feedback(T) arrives. A host stall then only makes the run slower: it can't change a verdict,
    and the bench's real-time guard no longer applies. Tests the logic and the CAN interface (DBC, E2E, ordering), not
    transport latency; real time stays the default for that.
    """
    realtime = True
    LOCKSTEP_TIMEOUT_S = 2.0

    def __init__(self, interface: str = "udp_multicast", channel: str | None = None, launch: str | list | None = "reference",
                 config_path: str = "config/default.json", status_replay: dict | None = None, status_e2e: bool = True,
                 lockstep: bool = False):
        import subprocess
        import sys
        import time

        from . import canio
        self.canio, self.time = canio, time
        self.db = canio.load_dbc()
        self.bus = canio.DedupBus(interface, channel or canio.GROUP)
        self.status_replay, self.status_e2e = status_replay, status_e2e
        self.lockstep, self.realtime = lockstep, not lockstep
        self.cycle_seen, self.status_c = 0, None
        self.inj = None
        if status_replay:
            self.inj = canio.DedupBus(interface, channel or canio.GROUP)
            self.inj.pid = f"{self.inj.pid}-replay"   # a second sender: its frames are not the bench's own
            self.inj.send(0x7FF, b"")   # warm up the socket outside the real-time loop (an ID nobody reads)
        self.status_rx = e2e.StatusReceiver()
        self.n_replays_sent = 0
        self.proc = None
        if launch == "reference":
            launch = [sys.executable, "-m", "ssb.vecu_process", "--interface", interface, "--channel", channel or canio.GROUP,
                      "--config", config_path]
        if launch:
            self.proc = subprocess.Popen(launch, cwd=str(Path(__file__).resolve().parent.parent))
        self.name = f"can:{interface}:{channel or canio.GROUP}" + (" (reference vECU process)" if self.proc else "")
        self.defects: frozenset = frozenset()
        self._wait_for(lambda m: m.arbitration_id == canio.ID["status"], 10.0, "vECU did not announce itself on CAN")
        self.last = Outputs()

    def _wait_for(self, pred, timeout_s: float, err: str):
        t_end = self.time.time() + timeout_s
        while self.time.time() < t_end:
            for m in self.bus.recv_all():
                if pred(m):
                    return m
            self.time.sleep(0.001)
        raise RuntimeError(err)

    def _ctrl(self, reset: int, power_ok: bool, tx_ok: bool, release: bool) -> None:
        from .safety import MUTANTS
        mask = sum(1 << k for k, n in enumerate(MUTANTS) if n in self.defects)
        self.bus.send(self.canio.ID["ctrl"], self.db.encode_message("BENCH_Control", {
            "BENCH_Reset": reset, "BENCH_PowerOk": int(power_ok), "BENCH_TxOk": int(tx_ok), "BENCH_Release": int(release),
            "BENCH_Lockstep": int(self.lockstep), "BENCH_Defects": mask}))

    def prepare(self, warm_start: bool, defects: frozenset = frozenset()) -> None:
        self.defects = frozenset(defects)
        self.bus.recv_all()
        self._ctrl(1 if warm_start else 2, True, True, False)
        want = self.canio.STATES.index("NORMAL" if warm_start else "INIT")
        if not self.lockstep:   # lockstep: the vECU acknowledges the reset itself; cycle 0 waits for step(0)'s inputs
            self._send_fb(0, {"v": 0.0, "a": 0.0, "delta": 0.0, "yaw_rate": 0.0, "grade_accel": 0.0})
        ack = self._wait_for(lambda m: m.arbitration_id == self.canio.ID["status"] and len(m.data) == 8 and m.data[2] & 0x07 == want,
                             5.0, "vECU did not reset")
        # real time: the reset's answer is cycle 0's status; lockstep: an acknowledgement before cycle 0 (counter 0)
        self.cycle_seen, self.status_c = (-1 if self.lockstep else 0), ack.data[1]
        self.ctrl_state, self.last = (True, True, False), Outputs(state="NORMAL" if warm_start else "INIT")
        self.status_rx = e2e.StatusReceiver()   # the vECU restarted its counter at the reset
        self.replay_frame, self.n_replays_sent = None, 0
        self._observe_reset()   # black box: everything the oracle needs comes from status frames seen on CAN

    def _send_fb(self, t: int, fb: dict) -> None:
        self.bus.send(self.canio.ID["fb"], self.db.encode_message("VEH_Feedback", {
            "VEH_Speed": fb["v"], "VEH_LongAccel": max(-327, min(327, fb["a"])), "VEH_RoadWheelAngle": fb["delta"],
            "VEH_YawRate": max(-3.27, min(3.27, fb["yaw_rate"])), "VEH_GradeAccel": fb.get("grade_accel", 0.0), "VEH_SimTime": t % 65536}), fd=True)

    def step(self, t, frames, kicks, fb, release, power_ok, tx_ok) -> Outputs:
        cid = self.canio.ID
        for f in frames:
            self.bus.send(f.can_id, f.data, fd=True)
        for _ in kicks:
            self.kick_n = (getattr(self, "kick_n", 0) + 1) & 0xFF
            self.bus.send(cid["kick"], bytes([self.kick_n]))
        state = (power_ok, tx_ok, release)
        if state != self.ctrl_state or t % 10 == 0:
            self._ctrl(0, power_ok, tx_ok, release)
            self.ctrl_state = state
        self._send_fb(t, fb)
        act = []
        st, cause, ch, mrm, cmd = self.last.state, self.last.cause, self.last.challenge, self.last.mrm_request, self.last.out_cmd
        msgs, self.checked = self.bus.recv_all(), {}
        if self.lockstep and t % 10 == 0:
            msgs = self._wait_cycle(t, msgs)
        for m in msgs:
            if m.arbitration_id == cid["act"]:
                act.append((m.arbitration_id, bytes(m.data)[:7]))
            elif m.arbitration_id == cid["status"]:
                data = bytes(m.data)
                if not self._accept(m):
                    continue   # corrupted, repeated or stale: not used
                self.last_status = data
                s = self.db.decode_message(m.arbitration_id, data, decode_choices=False)
                si, ci = int(s["SAF_State"]), int(s["SAF_Cause"])
                st = self.canio.STATES[si]
                cause = self.canio.CAUSES[ci] if ci < len(self.canio.CAUSES) else f"CAUSE_{ci}"
                ch, mrm = int(s["SAF_WdChallenge"]), ("PULL_OVER" if s["SAF_MrmRequest"] else None)
                cmd = (s["SAF_AccelOut"], s["SAF_SteerOut"], False)
        if self.status_replay:
            if t == self.status_replay["capture_ms"]:
                self.replay_frame = getattr(self, "last_status", None)
            if self.replay_frame and t in self.status_replay["inject_ms"]:
                self.inj.send(cid["status"], self.replay_frame)   # a stale copy, byte for byte, on the bus
                self.n_replays_sent += 1
        self._observe(t, self.last.state, st, cause, release)
        self.last = Outputs(act, st, cause, ch, mrm, cmd)
        return self.last

    def _count_cycle(self, data: bytes) -> None:
        """Unwrap the 8-bit alive counter into the vECU's cycle number since the reset (cycle 0 = the reset's answer)."""
        c = data[1]
        if self.status_c is not None:
            self.cycle_seen += (c - self.status_c) % 256
        self.status_c = c

    def _accept(self, m) -> bool:
        """E2E check of one status frame, once per frame (the lockstep wait and the step loop may both look at it);
        an accepted frame advances the cycle count."""
        if id(m) not in self.checked:
            data = bytes(m.data)
            ok = not self.status_e2e or self.status_rx.check(data)
            if ok:
                self._count_cycle(data)
            self.checked[id(m)] = ok
        return self.checked[id(m)]

    def _wait_cycle(self, t: int, msgs: list) -> list:
        """Lockstep: collect messages until the status of cycle t // 10 has arrived. Only frames that pass the E2E
        check count, so a replayed stale status can't release the wait. Returned in arrival order, handled as usual."""
        want, t_end, i = t // 10, self.time.perf_counter() + self.LOCKSTEP_TIMEOUT_S, 0
        while True:
            for m in msgs[i:]:
                if m.arbitration_id == self.canio.ID["status"]:
                    self._accept(m)
            i = len(msgs)
            if self.cycle_seen >= want:
                return msgs
            if self.time.perf_counter() > t_end:
                raise BenchFault(f"lockstep: no status for cycle {t} ms within {self.LOCKSTEP_TIMEOUT_S:g} s "
                                 f"(last cycle seen {self.cycle_seen * 10} ms): vECU stalled or crashed?")
            msgs += self.bus.recv_all(timeout=0.001)

    @property
    def n_status_crc(self) -> int:
        return self.status_rx.n_crc

    @property
    def n_status_seq(self) -> int:
        return self.status_rx.n_seq

    def close(self) -> None:
        if self.proc:
            self.proc.terminate()
            self.proc.wait(timeout=5)
        self.bus.shutdown()
        if self.inj:
            self.inj.shutdown()
