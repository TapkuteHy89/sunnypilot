from pathlib import Path
import os

import numpy as np

from openpilot.cereal.visionipc import VisionStreamType
from openpilot.common.swaglog import cloudlog


MI8_WIDE_CAMERA_EXTRINSICS_PATH = Path("/data/mi8_wide_from_device_euler.txt")


def is_mi8_hardware() -> bool:
  return os.path.isfile('/TICI') or os.path.isfile('/persist/comma/use-broadcom-gps')


def select_road_streams(available_streams, mi8_hardware: bool | None = None):
  """Select current narrow+wide inputs and identify the validated Mi 8 pair."""
  has_wide = VisionStreamType.VISION_STREAM_WIDE_ROAD in available_streams
  has_narrow = VisionStreamType.VISION_STREAM_NARROW_ROAD in available_streams
  if mi8_hardware is None:
    mi8_hardware = is_mi8_hardware()
  mi8_dual_camera = mi8_hardware and has_narrow and has_wide

  use_extra_client = has_wide and has_narrow
  main_wide_camera = not has_narrow
  if main_wide_camera:
    main_stream = VisionStreamType.VISION_STREAM_WIDE_ROAD
  else:
    main_stream = VisionStreamType.VISION_STREAM_NARROW_ROAD

  return main_stream, use_extra_client, main_wide_camera, mi8_dual_camera


def load_mi8_wide_camera_extrinsics(path: Path = MI8_WIDE_CAMERA_EXTRINSICS_PATH) -> np.ndarray | None:
  if not path.is_file():
    return None
  try:
    values = np.fromstring(path.read_text().strip(), dtype=np.float32, sep=" ")
  except OSError:
    cloudlog.exception("failed to read Mi 8 wide-camera extrinsics")
    return None
  if values.shape != (3,) or not np.isfinite(values).all() or np.any(np.abs(values) > np.radians(5.0)):
    cloudlog.error(f"ignoring invalid Mi 8 wide-camera extrinsics: {values}")
    return None
  return values
