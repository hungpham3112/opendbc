from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.vinfast import vinfastcan
from opendbc.car.vinfast.values import CarControllerParams

VisualAlert = structs.CarControl.HUDControl.VisualAlert

# Maximum steering angle (from /data/card values.h)
MAX_EPS_ANGLE = 470  # degrees


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0.0
    self.packer_chassis = CANPacker(dbc_names[Bus.chassis])
    # Note: Body CAN bus not accessible on comma3x, only Chassis and Camera buses
    self.frame = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []
    
    apply_angle = 0.0

    if CC.latActive:
      # VF8 uses angle-based steering control
      # Get desired steering angle from actuators (in degrees)
      desired_angle = CC.actuators.steeringAngleDeg
      
      # Apply angle rate limits (similar to /data/card implementation)
      # This prevents sudden angle changes
      if self.apply_angle_last != 0.0:
        # Simple rate limiting - can be improved with speed-based limits
        max_angle_change = 5.0  # degrees per step (TODO: calibrate based on speed)
        angle_diff = desired_angle - self.apply_angle_last
        angle_diff = max(-max_angle_change, min(max_angle_change, angle_diff))
        apply_angle = self.apply_angle_last + angle_diff
      else:
        apply_angle = desired_angle
      
      # Clamp to maximum angle
      apply_angle = max(-MAX_EPS_ANGLE, min(MAX_EPS_ANGLE, apply_angle))
      self.apply_angle_last = apply_angle

    # Send steering control
    if self.frame % 5 == 0:  # 20 Hz
      can_sends.append(vinfastcan.create_steering_control(
        self.packer_chassis,
        self.CP,
        self.frame,
        apply_angle,
        CC.latActive
      ))

    # Send ACC status (20 Hz)
    if self.frame % 5 == 0:
        accel_cmd = CC.actuators.accel if CC.longActive else 0.0
        can_sends.append(vinfastcan.create_acc_status(
            self.packer_chassis,
            self.CP,
            self.frame,
            accel_cmd,
            CC.longActive,
            CS.out.standstill
        ))

    new_actuators = CC.actuators.as_builder()
    # For angle control, steeringAngleDeg is already set by actuators
    new_actuators.steeringAngleDeg = apply_angle

    self.frame += 1
    return new_actuators, can_sends

