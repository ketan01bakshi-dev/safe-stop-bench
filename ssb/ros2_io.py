"""The bench's ROS 2 interface (v2.8): topic names, message types and payload layouts, shared by both sides.

Standard messages only (std_msgs), so neither side needs a colcon-built interface package. A supplied ROS 2 safety
node is wired in by remapping its topics onto these names (`--ros-args -r their_topic:=/ssb/...`); if it uses its own
message types, a small relay node converts them (see docs/ROS2_ADAPTER.md).

bench -> node
  /ssb/planner_cmd     std_msgs/UInt8MultiArray    the 14-byte PLN_Command frame, raw (CRC + counter + payload: E2E stays testable)
  /ssb/wd_kick         std_msgs/UInt32MultiArray   [kick counter (mod 256), sim_time_ms], one message per kick. The time
                                                   travels WITH the kick: topics are not ordered against each other, so a
                                                   kick stamped by the node with "latest feedback time" could be 1 ms early
  /ssb/vehicle_fb      std_msgs/Float64MultiArray  [sim_time_ms, v m/s, a m/s2, road-wheel deg, yaw rad/s, grade m/s2,
                                                    planner frames sent, kicks sent] (both since the reset). The node runs a
                                                    cycle only when it has received that many of each: no frame or kick can
                                                    slip to the next cycle because DDS delivered it after the feedback
  /ssb/bench_control   std_msgs/UInt32MultiArray   [reset (0 / 1 warm / 2 cold), power_ok, tx_ok, release, defects mask]
node -> bench
  /ssb/actuator_cmd    std_msgs/UInt8MultiArray    the 7-byte SAF_ActuatorCmd frame (Profile 2)
  /ssb/safety_status   std_msgs/Float64MultiArray  [sim_time_ms, state, cause, challenge, mrm_request, accel_out, steer_out]

Time: the bench owns it. The node runs its 10 ms task on the sim time carried by /ssb/vehicle_fb (like VEH_SimTime on
CAN), not on the ROS clock, so the timing it reports is comparable across adapters.
QoS: reliable, keep-last 100, volatile, on every topic. Both sides use the same profile, so they are compatible.
"""
from __future__ import annotations

PLANNER_CMD = "/ssb/planner_cmd"
WD_KICK = "/ssb/wd_kick"
VEHICLE_FB = "/ssb/vehicle_fb"
BENCH_CONTROL = "/ssb/bench_control"
ACTUATOR_CMD = "/ssb/actuator_cmd"
SAFETY_STATUS = "/ssb/safety_status"

STATES = ["INIT", "NORMAL", "DEGRADED", "PULL_OVER", "STOP_IN_LANE", "BRAKE_ONLY_STOP", "BACKUP_BRAKE_STOP", "OFF"]


def qos():
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=100,
                      durability=DurabilityPolicy.VOLATILE)
