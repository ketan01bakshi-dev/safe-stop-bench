# ROS 2 adapter (v2.8, 5 Oct 2026)

Closes item **1.1, ROS 2**. The safety controller runs as a **ROS 2 node in its own process**, reachable only through topics. The bench drives it as a black box, like the CAN vECU (v2.1) and the FMU (v2.2). Synthetic, illustrative limits.

## Setup (WSL)

ROS 2's Python client (`rclpy`) exists only on Linux, so this DUT runs in WSL. The bench core is pure standard-library Python, so it runs there unchanged.

```
wsl -d Ubuntu -- bash ros2/install_ros2_wsl.sh            # once: ROS 2 Lyrical ros-base (Ubuntu 26.04), official apt source
wsl -d Ubuntu -- bash ros2/run_in_wsl.sh python3 run.py --dut ros2 --ros2-lockstep --b2b-dut
wsl -d Ubuntu -- bash ros2/run_in_wsl.sh python3 ros2/exact_b2b.py        # exact on the 10 ms task grid
wsl -d Ubuntu -- bash ros2/run_in_wsl.sh python3 -m unittest tests.test_bench_v2.Ros2Tests
```

Installed: ROS 2 **Lyrical Luth** (`ros-lyrical-ros-base`), from the official `ros2-apt-source` package. Python 3.14 is the system Python; no venv is needed.

## Interface (`ssb/ros2_io.py`)

Standard `std_msgs` only, so neither side needs a colcon-built interface package. Reliable QoS, keep-last 100, on every topic.

| Topic | Type | Content |
|---|---|---|
| `/ssb/planner_cmd` | `UInt8MultiArray` | the 14-byte PLN_Command frame, **raw**: CRC and counter included, so E2E stays testable over ROS 2 |
| `/ssb/wd_kick` | `UInt32MultiArray` | `[kick counter, sim_time_ms]` |
| `/ssb/vehicle_fb` | `Float64MultiArray` | `[sim_time_ms, v, a, road-wheel angle, yaw rate, grade accel, frames sent, kicks sent]` |
| `/ssb/bench_control` | `UInt32MultiArray` | `[reset 0/1 warm/2 cold, power_ok, tx_ok, release, defects mask]` |
| `/ssb/actuator_cmd` | `UInt8MultiArray` | the 7-byte SAF_ActuatorCmd frame (Profile 2) |
| `/ssb/safety_status` | `Float64MultiArray` | `[cycle time ms, state, cause, challenge, MRM request, accel out, steer out]` |

**A supplied ROS 2 safety node** is wired in by remapping its topics (`--ros-args -r their/topic:=/ssb/...`) and running the bench with `--ros2-no-launch`. If it uses its own message types, a small relay node converts them. Its time base must be the bench's sim time (from `/ssb/vehicle_fb`), or the adapter runs in real time only.

## Two modes

- **Lockstep (`--ros2-lockstep`)**: not paced. At every 10 ms task boundary the bench waits for the node's status for that cycle. OS stalls can't change a verdict. This tests the logic and the ROS 2 interface (serialisation, QoS, ordering), not transport latency.
- **Real time (default)**: 1 simulated ms per wall-clock ms, so DDS transport and executor scheduling sit inside the measurement. It needs a quiet machine: the bench's lag guard fails any run it couldn't keep in time.

## Results

| Run (5 Oct 2026, WSL Ubuntu 26.04, ROS 2 Lyrical) | Result |
|---|---|
| Full matrix, lockstep (54 scenarios) | **53/54 + 1 known finding**, the same as every DUT |
| Back-to-back vs the in-process reference, lockstep | **54/54 match** (reaction, cause, detection) |
| **Exact back-to-back on the 10 ms task grid**, lockstep | **54/54 identical** (52/54 before the ordering fix, finding 1) |
| Real time, 6 key scenarios | 6/6 PASS on the second attempt, bench lag 0.2–2.7 ms; detection 51 ms (E2E, watchdog), 101 ms (timeout), 331 ms (steering). The first attempt failed the lag guard: 5.3–53 ms (finding 3) |
| `Ros2Tests` (in WSL) | 2/2: reactions vs reference, seeded bug `no_latch` caught inside the node |
| Bench step cost (real time) | mean 0.34 ms, p99 0.97 ms, max 11 ms per 1 ms step |
| Full lockstep matrix wall time | about 9 min |

## Findings

1. **Topics are not ordered against each other, and it changed a verdict's timing.** A watchdog kick carried no timestamp, so the node stamped it with "the latest feedback time". When DDS delivered the kick before that millisecond's feedback, the kick counted 1 ms early. The watchdog-late window then closed **one 10 ms cycle sooner** (`planner_hang`, `planner_power_dip`: 2040 vs 2050 ms; exact back-to-back 52/54). **Fix:** the kick carries its own sim time, and the feedback carries how many frames and kicks the bench had sent, so a cycle runs only once all of them have arrived (the FMU contract's Rx-counter idea). Exact 54/54 after. *Lesson: on CAN one bus keeps the order; in ROS 2 each topic is its own stream. Anything that must be ordered needs a timestamp or a sequence count in the message.*
2. **One topic = one CAN ID: the adapter must filter.** The first version forwarded every frame on the virtual planner bus to `/ssb/planner_cmd`, including a babbling node's junk frames. The node read them as corrupt commands and blamed E2E instead of the timeout (`bus_flood` FAIL). The flood's reliable backlog then leaked into the next scenario after the reset (`perception_degraded` went to E2E_INVALID at 10 ms). **Fix:** only PLN_Command frames go to the topic, and the node ignores feedback from before the reset.
3. **This laptop can't hold 5 ms real time, with or without ROS.** A bare busy loop, with no ROS and no bench, stalls up to **69 ms on Windows** (normal priority) and **up to 92 ms inside WSL**. The ROS 2 step itself costs 0.34 ms on average (p99 0.97 ms, max 11 ms). The lag guard correctly rejects real-time runs. Hence lockstep for verdicts, and real time only for latency on a quieter or real-time-patched machine. CPU pinning and disabling the garbage collector (`ssb/rt.py`, Linux branch) reduce but don't remove it.
4. **Raw frames over a topic keep E2E testable.** Because `/ssb/planner_cmd` carries the CRC and counter, every E2E scenario (corruption, counter faults, the v2.6 burst cases) runs unchanged over ROS 2.

## Honest limits

- `std_msgs` arrays, not a typed `.msg` package: fine for a bench contract, but a real project would define typed interfaces (and `can_msgs/Frame` for CAN-bridged topics).
- One DDS implementation (the distro default), loopback only, inside one WSL VM; no network, no QoS-mismatch tests yet.
- The node is my reference controller, so back-to-back proves the adapter and the transport, not an independent implementation.
- Real-time latency numbers from this laptop are not trustworthy (finding 3).
