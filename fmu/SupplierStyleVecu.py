"""SupplierStyleVecu: the SAME controller, exported the way a supplier might: their own variable names, vehicle
speed in km/h, a different state enumeration, and no bench hooks (no mutants, no config switch, no warm start).

It exists to test the bench's mapping path: the bench can only drive this FMU through a mapping file
(fmu/mapping_supplier_style.json, proposed by `python -m ssb.fmu_inspect ... --write-mapping` and then corrected by hand).
"""
from SafeStopVecu import SafeStopVecu


class SupplierStyleVecu(SafeStopVecu):
    description = "Safety controller, supplier-style export (illustrative)"
    NAMES = {
        "Bench_WarmStart": None, "Bench_Defects": None, "Bench_Config": None,   # supplier FMUs carry no test hooks
        "PLN_Command_RxCounter": "Com_PlnCmd_RxCnt", "PLN_Command_DLC": "Com_PlnCmd_Dlc",
        "PLN_E2E_CRC": "PlnCmd_Crc", "PLN_E2E_Counter": "PlnCmd_AliveCtr", "PLN_TimeStamp": "PlnCmd_Tstamp",
        "PLN_AccelReq": "PlnCmd_AxReq", "PLN_SteerReq": "PlnCmd_SteerAngReq", "PLN_SpeedReq": "PlnCmd_VReq",
        "PLN_WdAnswer": "PlnCmd_WdgResp", "PLN_PerceptionHealth": "PlnCmd_PercQly", "PLN_Flags": "PlnCmd_Flg",
        "PLN_WdKickCounter": "Dio_WdgTrigCnt",
        "VEH_Speed": "VehSpd_kph", "VEH_LongAccel": "VehAx", "VEH_RoadWheelAngle": "RoadWhlAng", "VEH_YawRate": "YawRate",
        "VEH_GradeAccel": "RoadGradeAx",
        "Bench_PowerOk": "Pwr_SupplyOk", "Bench_TxOk": "Can_ActBusOk", "Bench_Release": "Hmi_OperatorRelease",
        "SAF_ActuatorCmd_TxCounter": "Com_ActCmd_TxCnt", "SAF_E2E_CRC": "ActCmd_Crc", "SAF_E2E_Counter": "ActCmd_AliveCtr",
        "SAF_AccelCmd": "ActCmd_AxCmd", "SAF_SteerCmd": "ActCmd_SteerAngCmd", "SAF_BackupBrake": "ActCmd_BkpBrkReq",
        "SAF_State": "SfmState", "SAF_Cause": "SfmFaultReason", "SAF_WdChallenge": "Wdg_Challenge",
        "SAF_MrmRequest": "Mrm_Req", "SAF_AccelOut": "Dbg_AxOut", "SAF_SteerOut": "Dbg_SteerOut",
    }
    SPEED_UNIT = 3.6
    STATE_CODES = ["OFF", "INIT", "NORMAL", "DEGRADED", "PULL_OVER", "STOP_IN_LANE", "BRAKE_ONLY_STOP", "BACKUP_BRAKE_STOP"]
