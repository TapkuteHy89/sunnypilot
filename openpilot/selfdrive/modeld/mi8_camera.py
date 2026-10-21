import os

import numpy as np

from openpilot.cereal.visionipc import VisionStreamType
from openpilot.common.transformations.orientation import euler_from_rot, rot_from_euler


MAX_WIDE_CAMERA_ANGLE = np.radians(5.0)


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


def get_live_wide_camera_extrinsics(extrinsics_calibration) -> np.ndarray | None:
  values = np.asarray(extrinsics_calibration.wideFromDeviceEuler, dtype=np.float32)
  if values.shape != (3,) or not np.isfinite(values).all() or np.any(np.abs(values) > MAX_WIDE_CAMERA_ANGLE):
    return None
  return values


def compose_wide_camera_extrinsics(current: np.ndarray | None, residual: np.ndarray) -> np.ndarray:
  """Convert the model residual into the absolute wide-from-device rotation."""
  residual = np.asarray(residual, dtype=np.float32)
  if current is None:
    return residual
  return euler_from_rot(rot_from_euler(current) @ rot_from_euler(residual)).astype(np.float32)
