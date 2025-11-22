from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.vinfast.carcontroller import CarController
from opendbc.car.vinfast.carstate import CarState
from opendbc.car.vinfast.values import CAR


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "vinfast"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.vinfast)]
    
    # Angle-based lateral control setup (VF8 uses angle controller)
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.8  # Longer timer for angle control
    ret.lateralTuning.init('angle')
    ret.lateralTuning.angle.kpBP = [0.]
    ret.lateralTuning.angle.kpV = [1.0]
    ret.lateralTuning.angle.kf = 1.0
    ret.centerToFront = ret.wheelbase * 0.4

    # Common longitudinal control setup
    ret.radarUnavailable = True  # TODO: check if radar is available
    ret.openpilotLongitudinalControl = alpha_long
    ret.pcmCruise = not ret.openpilotLongitudinalControl
    ret.startingState = True
    ret.vEgoStarting = 0.1
    ret.startAccel = 1.0
    ret.longitudinalActuatorDelay = 0.5

    # Car specific configuration
    ret.dashcamOnly = False  # TODO: set to True if not fully tested

    return ret

