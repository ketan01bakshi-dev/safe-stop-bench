"""The reference safety controller as a ROS 2 node (v2.8): a stand-in for a supplier's safety node.

    source /opt/ros/<distro>/setup.bash
    python3 -m ssb.ros2_vecu_node [--config config/default.json]

Reachable ONLY through the topics in ssb/ros2_io.py. Same logic as ssb/vecu_process.py (the CAN vECU): the bench resets
it over /ssb/bench_control, drives time through /ssb/vehicle_fb, and reads actuator commands and status back.
Until the first reset it announces itself with an OFF status every 200 ms, so the bench knows the node is up.
"""
from __future__ import annotations

import argparse
import time

from . import config, ros2_io
from .safety import CYCLE_MS, MUTANTS, SafetyController

CAUSES = [None, "TIMEOUT", "E2E_INVALID", "STALE_DATA", "WATCHDOG_LATE", "WATCHDOG_EARLY", "WATCHDOG_QA", "ENVELOPE",
          "STEER_ACTUATOR", "BRAKE_ACTUATOR", "ACT_BUS_OFF", "PERCEPTION_DEGRADED", "ODD_EXIT", "PERCEPTION_LOST", "SAFETY_RESET"]


class _Frame:
    def __init__(self, can_id: int, data: bytes):
        self.can_id, self.data = can_id, data


class SafetyNode:
    """Topic callbacks around SafetyController. Time = the bench's sim time from /ssb/vehicle_fb."""

    def __init__(self, cfg: dict, pub_act, pub_status, msg_types):
        self.cfg, self.pub_act, self.pub_status = cfg, pub_act, pub_status
        self.F64, self.U8A = msg_types
        self.sc: SafetyController | None = None
        self.t = self.next_cycle = 0
        self.pending: list[_Frame] = []
        self.power_ok = self.tx_ok = True
        self.last_release = 0
        self.fresh = False   # after a reset: ignore feedback until this run's first one (t = 0)
        # Topics are not ordered against each other. The feedback carries how many planner frames and kicks the bench
        # had sent by then; a cycle runs only once all of them have arrived (the FMU contract's Rx-counter idea).
        self.rx_cmd = self.rx_kick = self.need_cmd = self.need_kick = 0
        self.fb = {"v": 0.0, "a": 0.0, "delta": 0.0, "yaw_rate": 0.0, "grade_accel": 0.0}

    def status(self, t: int) -> None:
        msg = self.F64()
        sc = self.sc
        if sc is None:
            msg.data = [float(t), 7.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            a_out, s_out, _ = sc.out
            msg.data = [float(t), float(ros2_io.STATES.index(sc.state if sc.powered else "OFF")),
                        float(CAUSES.index(sc.cause) if sc.cause in CAUSES else 0), float(sc.challenge),
                        1.0 if sc.mrm_request == "PULL_OVER" else 0.0, float(a_out), float(s_out)]
        self.pub_status.publish(msg)

    def on_control(self, m) -> None:
        reset, power_ok, tx_ok, release, mask = (int(x) for x in m.data)
        if reset:
            defects = frozenset(n for k, n in enumerate(MUTANTS) if mask >> k & 1)
            self.sc = SafetyController(self.cfg, defects, warm_start=reset == 1)
            self.t = self.next_cycle = self.last_release = 0
            self.pending, self.fresh = [], False
            self.rx_cmd = self.rx_kick = self.need_cmd = self.need_kick = 0
            self.status(0)                       # acknowledge the reset at once
        if self.sc is None:
            return
        if release and not self.last_release:
            self.sc.release(self.t, self.fb["v"])
        self.last_release, self.power_ok, self.tx_ok = release, bool(power_ok), bool(tx_ok)

    def on_fb(self, m) -> None:
        if self.sc is None:
            return
        t, v, acc, delta, yaw, grade, n_cmd, n_kick = m.data
        if not self.fresh:
            if int(t) != 0:      # stale feedback from the previous run (the bench's prepare() sends t = 0 first)
                return
            self.fresh = True
        self.t = max(self.t, int(t))             # a late or duplicated message never moves time backwards
        self.fb = {"v": v, "a": acc, "delta": delta, "yaw_rate": yaw, "grade_accel": grade}
        self.need_cmd, self.need_kick = max(self.need_cmd, int(n_cmd)), max(self.need_kick, int(n_kick))
        self.run_cycles()

    def on_kick(self, m) -> None:
        if self.sc is not None:
            _, t = m.data
            self.sc.kick(int(t))         # the kick's own sim time (it may arrive before that millisecond's feedback)
            self.rx_kick += 1
            self.run_cycles()

    def on_cmd(self, m) -> None:
        if self.sc is not None:
            self.pending.append(_Frame(0x100, bytes(m.data)))
            self.rx_cmd += 1
            self.run_cycles()

    def run_cycles(self) -> None:
        sc = self.sc
        if sc is None or not self.fresh or self.rx_cmd < self.need_cmd or self.rx_kick < self.need_kick:
            return                       # something the bench sent before this feedback is still in flight
        sc.brownout(self.t, not self.power_ok)
        sc.tx_ok = self.tx_ok
        while self.next_cycle <= self.t:
            for _, data in sc.cycle(self.next_cycle, self.pending, self.fb):
                out = self.U8A()
                out.data = list(data)
                self.pub_act.publish(out)
            self.pending = []
            self.status(self.next_cycle)
            self.next_cycle += CYCLE_MS


def main() -> None:
    import rclpy
    import rclpy.executors
    from std_msgs.msg import Float64MultiArray, UInt8MultiArray, UInt32MultiArray

    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/default.json")
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = rclpy.create_node("ssb_safety_controller")
    q = ros2_io.qos()
    sn = SafetyNode(config.load(a.config), node.create_publisher(UInt8MultiArray, ros2_io.ACTUATOR_CMD, q),
                    node.create_publisher(Float64MultiArray, ros2_io.SAFETY_STATUS, q), (Float64MultiArray, UInt8MultiArray))
    node.create_subscription(UInt32MultiArray, ros2_io.BENCH_CONTROL, sn.on_control, q)
    node.create_subscription(Float64MultiArray, ros2_io.VEHICLE_FB, sn.on_fb, q)
    node.create_subscription(UInt32MultiArray, ros2_io.WD_KICK, sn.on_kick, q)
    node.create_subscription(UInt8MultiArray, ros2_io.PLANNER_CMD, sn.on_cmd, q)
    import gc
    gc.disable()                                  # no collector pauses inside the 10 ms task (see ssb/rt.py)
    hello = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0005)
            if sn.sc is None and time.monotonic() - hello > 0.2:
                sn.status(0)
                hello = time.monotonic()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as e:  # noqa: BLE001
        if rclpy.ok():                            # a real error; after SIGTERM the context is gone: not one
            raise
        del e
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
