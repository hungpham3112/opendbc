""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.vinfast.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.VINFAST_VF8: {
    # TODO: Add firmware versions when available
    # Example structure:
    # (Ecu.eps, 0x730, None): [
    #   b'FW_VERSION_STRING\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    # ],
  },
}

