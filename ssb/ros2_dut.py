"""Ros2DUT (v2.8): a safety node in a SEPARATE PROCESS, reachable only over ROS 2 topics (rclpy, std_msgs).

    source /opt/ros/<distro>/setup.bash          # in WSL / Linux: rclpy is not on the Windows Python
    python3 run.py --dut ros2 [--b2b-dut]

By default it launches our reference controller as that node (`python3 -m ssb.ros2_vecu_node`), which makes it a
true black box and lets back-to-back runs prove the adapter. For a supplied ROS 2 safety node, pass launch=None (CLI:
--ros2-no-launch) and start theirs with its topics remapped onto ssb/ros2_io.py.
Two modes:
- real time (default), like CanDUT: the runner paces 1 simulated ms per wall-clock ms; DDS transport and executor
  scheduling are inside the measurement. Needs a quiet machine: the bench's lag guard fails runs it can't keep in time.
- lockstep (--ros2-lockstep): not paced; at every 10 ms task boundary the bench waits until the node has published
  that cycle's status. The node runs on the bench's sim time anyway, so OS stalls can't change a verdict. Tests the
  logic and the ROS 2 interface (serialisation, QoS, topic ordering), not transport latency. Each test run gets its own ROS_DOMAIN_ID unless one is set, so parallel runs don't cross-talk.
"""
from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

from . import ros2_io
from .dut import BlackBoxObserver, DeviceUnderTest, Outputs

ROOT = Path(__file__).resolve().parent.parent
CAUSES = [None, "TIMEOUT", "E2E_INVALID", "STALE_DATA", "WATCHDOG_LATE", "WATCHDOG_EARLY", "WATCHDOG_QA", "ENVELOPE",
          "STEER_ACTUATOR", "BRAKE_ACTUATOR", "ACT_BUS_OFF", "PERCEPTION_DEGRADED", "ODD_EXIT", "PERCEPTION_LOST", "SAFETY_RESET"]


class Ros2DUT(BlackBoxObserver, DeviceUnderTest):
    realtime = True

    def __init__(self, launch: str | list | None = "reference", config_path: str = "config/default.json", lockstep: bool = False):
        self.lockstep, self.realtime = lockstep, not lockstep
        try:
            import rclpy
            from std_msgs.msg import Float64MultiArray, UInt8MultiArray, UInt32MultiArray
        except ImportError as e:  # pragma: no cover
            raise RuntimeError("Ros2DUT needs rclpy: run in WSL/Linux after 'source /opt/ros/<distro>/setup.bash'") from e
        self.msg = {"f64": Float64MultiArray, "u8a": UInt8MultiArray, "u32a": UInt32MultiArray}
        os.environ.setdefault("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))   # isolate from other ROS 2 graphs
        if not rclpy.ok():
            rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node("ssb_bench")
        q = ros2_io.qos()
        self.pub = {
            "cmd": self.node.create_publisher(UInt8MultiArray, ros2_io.PLANNER_CMD, q),
            "kick": self.node.create_publisher(UInt32MultiArray, ros2_io.WD_KICK, q),
            "fb": self.node.create_publisher(Float64MultiArray, ros2_io.VEHICLE_FB, q),
            "ctrl": self.node.create_publisher(UInt32MultiArray, ros2_io.BENCH_CONTROL, q),
        }
        self.inbox: list[tuple[str, object]] = []
        self.node.create_subscription(UInt8MultiArray, ros2_io.ACTUATOR_CMD, lambda m: self.inbox.append(("act", m)), q)
        self.node.create_subscription(Float64MultiArray, ros2_io.SAFETY_STATUS, lambda m: self.inbox.append(("status", m)), q)
        self.proc = None
        if launch == "reference":
            launch = [sys.executable, "-m", "ssb.ros2_vecu_node", "--config", config_path]
        if launch:
            from . import rt
            cpus = rt.dut_cpus()      # the node on the other half of the cores (bench_cpus() for the bench, rt.boost)
            setaff = getattr(os, "sched_setaffinity", None)   # Linux only
            pin = (lambda: setaff(0, cpus)) if cpus and setaff else None
            self.proc = subprocess.Popen(launch, cwd=str(ROOT), env=os.environ.copy(), preexec_fn=pin)
        self.name = (f"ros2:domain {os.environ['ROS_DOMAIN_ID']}" + (" lockstep" if lockstep else " real time")
                     + (" (reference node process)" if self.proc else ""))
        self.defects: frozenset = frozenset()
        self.kick_n = self.n_cmd = self.n_kick = 0
        self._wait_status(lambda s: True, 20.0, "safety node did not announce itself on /ssb/safety_status")
        self.last = Outputs()

    # ---- helpers ---------------------------------------------------------------------------------------------------
    def _spin(self) -> list:
        for _ in range(50):                   # drain whatever has arrived, without blocking
            n = len(self.inbox)
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
            if len(self.inbox) == n:
                break
        out, self.inbox = self.inbox, []
        return out

    def _wait_status(self, pred, timeout_s: float, err: str):
        t_end = time.monotonic() + timeout_s
        while time.monotonic() < t_end:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)
            for kind, m in self._spin_take():
                if kind == "status" and pred(list(m.data)):
                    return m
        raise RuntimeError(err)

    def _spin_take(self) -> list:
        out, self.inbox = self.inbox, []
        return out

    def _ctrl(self, reset: int, power_ok: bool, tx_ok: bool, release: bool) -> None:
        from .safety import MUTANTS
        m = self.msg["u32a"]()
        m.data = [reset, int(power_ok), int(tx_ok), int(release), sum(1 << k for k, n in enumerate(MUTANTS) if n in self.defects)]
        self.pub["ctrl"].publish(m)

    def _send_fb(self, t: int, fb: dict) -> None:
        m = self.msg["f64"]()
        m.data = [float(t), fb["v"], fb["a"], fb["delta"], fb["yaw_rate"], fb.get("grade_accel", 0.0),
                  float(self.n_cmd), float(self.n_kick)]
        self.pub["fb"].publish(m)

    # ---- DeviceUnderTest -------------------------------------------------------------------------------------------
    def prepare(self, warm_start: bool, defects: frozenset = frozenset()) -> None:
        self.defects = frozenset(defects)
        self.n_cmd = self.n_kick = 0    # sent since the reset; carried in every feedback (see ssb/ros2_io.py)
        self._spin()
        want = float(ros2_io.STATES.index("NORMAL" if warm_start else "INIT"))
        self._ctrl(1 if warm_start else 2, True, True, False)
        self._wait_status(lambda s: s[0] == 0.0 and s[1] == want, 5.0, "safety node did not reset")
        self._send_fb(0, {"v": 0.0, "a": 0.0, "delta": 0.0, "yaw_rate": 0.0, "grade_accel": 0.0})
        self.ctrl_state, self.last = (True, True, False), Outputs(state="NORMAL" if warm_start else "INIT")
        self._observe_reset()

    def step(self, t, frames, kicks, fb, release, power_ok, tx_ok) -> Outputs:
        for f in frames:
            if f.can_id != 0x100:     # one topic = one CAN ID: only PLN_Command goes to /ssb/planner_cmd (a babbler's
                continue              # frames only matter on CAN, where they delay the real ones; the bus model does that)
            m = self.msg["u8a"]()
            m.data = list(f.data[:14])
            self.pub["cmd"].publish(m)
            self.n_cmd += 1
        for _ in kicks:
            self.kick_n = (self.kick_n + 1) & 0xFF
            k = self.msg["u32a"]()
            k.data = [self.kick_n, t]
            self.pub["kick"].publish(k)
            self.n_kick += 1
        state = (power_ok, tx_ok, release)
        if state != self.ctrl_state or t % 10 == 0:
            self._ctrl(0, power_ok, tx_ok, release)
            self.ctrl_state = state
        self._send_fb(t, fb)
        act = []
        st, cause, ch, mrm, cmd = self.last.state, self.last.cause, self.last.challenge, self.last.mrm_request, self.last.out_cmd
        msgs = self._spin()
        if self.lockstep and t % 10 == 0:       # wait for the node's cycle t (status stamped with its cycle time)
            t_end = time.monotonic() + 2.0
            while not any(k == "status" and m.data[0] >= t for k, m in msgs):
                if time.monotonic() > t_end:
                    raise RuntimeError(f"lockstep: no status for cycle {t} ms within 2 s (node stalled or crashed?)")
                self.rclpy.spin_once(self.node, timeout_sec=0.001)
                msgs += self._spin_take()
        for kind, m in msgs:
            if kind == "act":
                act.append((0x200, bytes(m.data)))
            else:
                s = list(m.data)
                st, cause = ros2_io.STATES[int(s[1])], CAUSES[int(s[2])]
                ch, mrm, cmd = int(s[3]), ("PULL_OVER" if s[4] else None), (s[5], s[6], False)
        self._observe(t, self.last.state, st, cause, release)
        self.last = Outputs(act, st, cause, ch, mrm, cmd)
        return self.last

    def identity(self) -> str:
        return self.name

    def close(self) -> None:
        if self.proc:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.node.destroy_node()
