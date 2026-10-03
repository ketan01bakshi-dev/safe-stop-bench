"""The bench's FMU contract: the variables a safety-controller FMU must (or may) expose, by BENCH name.

A supplied FMU rarely uses our names. A mapping file (JSON) maps each bench name to the FMU's variable name, so a
supplier's FMU is wired in without code changes. `python -m ssb.fmu_inspect their.fmu --write-mapping m.json`
proposes that mapping from the FMU's modelDescription.xml.

Why signal level, not frames: FMI 2.0 has no binary type, and supplier vECU FMUs (exported from Simulink, TargetLink,
Silver, VEOS, or AUTOSAR "COM-level" builds) normally expose decoded signals. We pass the DBC signals as RAW integers
(no scaling), so the 14-byte planner frame survives the trip bit for bit and the FMU can still check E2E on it.
Frames are events and FMI 2.0 co-simulation has no events, so each message carries an Rx/Tx COUNTER: the receiver
sees a new frame when the counter changes. (FMI 3.0 adds Binary variables and clocks; FMI-LS-BUS standardises
frame-level CAN on top. Not needed for FMI 2.0 FMUs, which are still the common delivery.)
"""
from __future__ import annotations

# (bench name, FMI type, required, what it is)
INPUTS = [
    ("PLN_Command_RxCounter", "Integer", True, "increments by 1 for every PLN_Command frame received (event)"),
    ("PLN_Command_DLC", "Integer", False, "received length in bytes (14 = full frame); shorter frames are truncated"),
    ("PLN_E2E_CRC", "Integer", True, "raw DBC signal"),
    ("PLN_E2E_Counter", "Integer", True, "raw DBC signal"),
    ("PLN_TimeStamp", "Integer", True, "raw DBC signal, ms"),
    ("PLN_AccelReq", "Integer", True, "raw DBC signal, 0.01 m/s2, signed"),
    ("PLN_SteerReq", "Integer", True, "raw DBC signal, 0.01 deg, signed"),
    ("PLN_SpeedReq", "Integer", True, "raw DBC signal, 0.01 m/s"),
    ("PLN_WdAnswer", "Integer", True, "raw DBC signal"),
    ("PLN_PerceptionHealth", "Integer", True, "raw DBC signal"),
    ("PLN_Flags", "Integer", True, "raw DBC signal"),
    ("PLN_WdKickCounter", "Integer", True, "increments by 1 per hardware-watchdog kick (mod 256)"),
    ("VEH_Speed", "Real", True, "m/s"),
    ("VEH_LongAccel", "Real", True, "m/s2"),
    ("VEH_RoadWheelAngle", "Real", True, "deg"),
    ("VEH_YawRate", "Real", True, "rad/s"),
    ("VEH_GradeAccel", "Real", True, "m/s2"),
    ("Bench_PowerOk", "Boolean", False, "false = supply brown-out (fault injection)"),
    ("Bench_TxOk", "Boolean", False, "false = actuator bus-off (fault injection)"),
    ("Bench_Release", "Boolean", False, "operator release request (true for one step)"),
]
OUTPUTS = [
    ("SAF_ActuatorCmd_TxCounter", "Integer", True, "increments by 1 for every SAF_ActuatorCmd frame sent (event)"),
    ("SAF_E2E_CRC", "Integer", True, "raw DBC signal"),
    ("SAF_E2E_Counter", "Integer", True, "raw DBC signal"),
    ("SAF_AccelCmd", "Integer", True, "raw DBC signal, 0.01 m/s2, signed"),
    ("SAF_SteerCmd", "Integer", True, "raw DBC signal, 0.01 deg, signed"),
    ("SAF_BackupBrake", "Integer", True, "raw DBC signal"),
    ("SAF_State", "Integer", True, "enumeration; default values as in the DBC (VAL_ 513 SAF_State)"),
    ("SAF_Cause", "Integer", False, "first fault cause, enumeration as in ssb.canio.CAUSES"),
    ("SAF_WdChallenge", "Integer", True, "question-and-answer watchdog challenge for the planner"),
    ("SAF_MrmRequest", "Integer", False, "1 = ask the planner to pull over"),
    ("SAF_AccelOut", "Real", False, "m/s2 the controller is commanding (for invariants)"),
    ("SAF_SteerOut", "Real", False, "deg the controller is commanding (for invariants)"),
]
PARAMETERS = [  # set before initialisation; bench hooks, a supplier FMU normally has none of these
    ("Bench_WarmStart", "Boolean", False, "true = start in NORMAL (vehicle already moving), false = INIT"),
    ("Bench_Defects", "Integer", False, "bitmask of seeded mutants (reference FMU only), order of ssb.safety.MUTANTS"),
    ("Bench_Config", "String", False, "which bundled config the reference FMU uses (default / offroad)"),
]
ALL = {n: (kind, typ, req, doc) for kind, rows in (("input", INPUTS), ("output", OUTPUTS), ("parameter", PARAMETERS))
       for n, typ, req, doc in rows}

PLN_SIGNALS = ["PLN_E2E_CRC", "PLN_E2E_Counter", "PLN_TimeStamp", "PLN_AccelReq", "PLN_SteerReq", "PLN_SpeedReq",
               "PLN_WdAnswer", "PLN_PerceptionHealth", "PLN_Flags"]
SAF_SIGNALS = ["SAF_E2E_CRC", "SAF_E2E_Counter", "SAF_AccelCmd", "SAF_SteerCmd", "SAF_BackupBrake"]
STATE_VALUES = {0: "INIT", 1: "NORMAL", 2: "DEGRADED", 3: "PULL_OVER", 4: "STOP_IN_LANE", 5: "BRAKE_ONLY_STOP",
                6: "BACKUP_BRAKE_STOP", 7: "OFF"}


def identity_mapping() -> dict:
    """The mapping for an FMU that uses the bench names (our reference FMU)."""
    return {"inputs": {n: n for n, *_ in INPUTS}, "outputs": {n: n for n, *_ in OUTPUTS},
            "parameters": {n: n for n, *_ in PARAMETERS}, "state_values": {str(k): v for k, v in STATE_VALUES.items()}}
