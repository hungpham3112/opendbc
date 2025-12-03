from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class CarControllerParams:
  ACCEL_MIN = -3.5  # m/s
  ACCEL_MAX = 2.0   # m/s

  STEER_MAX = 470
  STEER_DELTA_UP = 3
  STEER_DELTA_DOWN = 7
  STEER_DRIVER_ALLOWANCE = 50
  STEER_DRIVER_MULTIPLIER = 2
  STEER_DRIVER_FACTOR = 1
  STEER_THRESHOLD = 150
  STEER_STEP = 1  # 100 Hz

  def __init__(self, CP):
    pass


class VinFastFlags(IntFlag):
  # Static flags
  # Add flags as needed
  pass


@dataclass
class VinFastCarDocs(CarDocs):
  package: str = "All"


@dataclass
class VinFastPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.chassis: "vinfast_vf8_chassis_can",
    Bus.cam: "vinfast_vf8_chassis_can",  # Camera bus uses same DBC (SCAM messages)
  })


class CAR(Platforms):
  VINFAST_VF8 = VinFastPlatformConfig(
    [VinFastCarDocs("VinFast VF8 2023-24", "All", car_parts=CarParts.common([CarHarness.custom]))],
    CarSpecs(mass=2200, wheelbase=2.95, steerRatio=15.828, tireStiffnessFactor=0.82),
  )


class CANBUS:
  # Note: Due to wiring, bus assignments are reversed:
  # - Bus 2 = Chassis bus (physical chassis CAN)
  # - Bus 0 = SCAM bus (camera/SCAM CAN)
  chassis = 2  # Bus 2 is chassis bus (due to wiring)
  cam = 0      # Bus 0 is SCAM/camera bus (due to wiring)

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()

