#!/usr/bin/env python3
import json
import os

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params


def main() -> None:
  params = Params(os.getenv("PARAMS_PATH", ""))
  raw = params.get("CalibrationParams")
  previous = messaging.log_from_bytes(raw).extrinsicsCalibration if raw else None

  msg = messaging.new_message("extrinsicsCalibration")
  calibration = msg.extrinsicsCalibration
  calibration.validBlocks = 0
  calibration.rpyCalib = list(previous.rpyCalib) if previous is not None and len(previous.rpyCalib) == 3 else [0.0, 0.0, 0.0]
  calibration.rpyCalibSpread = [0.0, 0.0, 0.0]
  calibration.wideFromDeviceEuler = [0.0, 0.0, 0.0]
  calibration.height = list(previous.height) if previous is not None and len(previous.height) == 1 else [1.22]
  params.put("CalibrationParams", msg.to_bytes(), block=True)

  print(json.dumps({
    "previousValidBlocks": previous.validBlocks if previous else None,
    "previousRpyCalib": list(previous.rpyCalib) if previous else None,
    "previousWideFromDeviceEuler": list(previous.wideFromDeviceEuler) if previous else None,
    "newValidBlocks": calibration.validBlocks,
    "newRpyCalib": list(calibration.rpyCalib),
    "newWideFromDeviceEuler": list(calibration.wideFromDeviceEuler),
  }, indent=2))


if __name__ == "__main__":
  main()
