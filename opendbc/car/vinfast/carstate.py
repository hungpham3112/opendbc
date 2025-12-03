from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.vinfast.values import DBC


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.can_parsers = []

  def get_can_parsers(self, CP):
    ret = {}

    # Chassis bus (steering, brakes, etc.)
    # CANParser expects: list of (message_name, frequency) tuples
    messages = [
      ("SAS_Sensor", 50),
      ("EPS_ADAS_Steering_Trq", 50),
      ("IDB_STATUS", 50),
      ("IDB_AVL_RPM_WHL_FRONT", 50),
      ("IDB_AVL_RPM_WHL_REAR", 50),
      ("ADAS_ACC_Status", 50),
      ("EPS_SteeringHoldState", 50),
    ]

    # Use physical bus number (CANBUS.chassis = 2)
    from opendbc.car.vinfast.values import CANBUS
    ret[Bus.chassis] = CANParser(DBC[CP.carFingerprint][Bus.chassis], messages, CANBUS.chassis)

    # Note: Info and Body CAN buses are not accessible on comma3x
    # Only Chassis and Camera buses are available
    # Gear, doors, and other body signals are not accessible

    return ret

  def update(self, can_parsers) -> structs.CarState:
    cp_chassis = can_parsers[Bus.chassis]

    ret = structs.CarState()

    # Wheel speeds (convert from rad/s to m/s)
    # TODO: verify the conversion factor
    fl_wheel_speed = cp_chassis.vl["IDB_AVL_RPM_WHL_FRONT"]["AVL_RPM_WHL_FLH"] * 0.3  # approximate conversion
    fr_wheel_speed = cp_chassis.vl["IDB_AVL_RPM_WHL_FRONT"]["AVL_RPM_WHL_FRH"] * 0.3
    rl_wheel_speed = cp_chassis.vl["IDB_AVL_RPM_WHL_REAR"]["AVL_RPM_WHL_RLH"] * 0.3
    rr_wheel_speed = cp_chassis.vl["IDB_AVL_RPM_WHL_REAR"]["AVL_RPM_WHL_RRH"] * 0.3

    self.parse_wheel_speeds(ret, fl_wheel_speed, fr_wheel_speed, rl_wheel_speed, rr_wheel_speed)

    # Vehicle speed
    vehicle_speed_kph = cp_chassis.vl["IDB_STATUS"]["VehicleSpd"]
    ret.vEgo = vehicle_speed_kph * CV.KPH_TO_MS
    ret.vEgoRaw = ret.vEgo
    ret.standstill = cp_chassis.vl["IDB_STATUS"]["ESC_VehicleStandstill"] == 1

    # Steering
    ret.steeringAngleDeg = cp_chassis.vl["SAS_Sensor"]["SAS_SteerWheelAngle"]
    ret.steeringTorque = cp_chassis.vl["EPS_ADAS_Steering_Trq"]["EPS_SteeringDriverTorque"]
    ret.steeringTorqueEps = cp_chassis.vl["EPS_ADAS_Steering_Trq"]["EPS_SteeringEMTorque"]
    ret.steeringPressed = abs(ret.steeringTorque) > 50  # TODO: calibrate threshold
    ret.steeringRateDeg = cp_chassis.vl["SAS_Sensor"]["SAS_SteerWhlRotSpd"]

    # Gear - not accessible on comma3x (Body CAN not connected)
    # Set to unknown as fallback
    ret.gearShifter = structs.CarState.GearShifter.unknown

    # Doors - not accessible on comma3x (Body CAN not connected)
    # Set to False as fallback
    ret.doorOpen = False

    # Cruise control
    acc_mode = cp_chassis.vl["ADAS_ACC_Status"]["ADAS_ACC_Mode"]
    acc_main_mode = cp_chassis.vl["ADAS_ACC_Status"]["ADAS_ACC_Main_Mode"]
    ret.cruiseState.available = acc_main_mode == 1
    ret.cruiseState.enabled = acc_mode != 0
    ret.cruiseState.standstill = ret.standstill

    # EPS status
    eps_available = cp_chassis.vl["EPS_SteeringHoldState"]["EPS_ADAS_ControlAvailable"] == 1
    eps_intervention = cp_chassis.vl["EPS_SteeringHoldState"]["EPS_Drive_Intervention"] == 1

    # TODO: Add more signals as needed (brake, gas, seatbelt, etc.)

    return ret

