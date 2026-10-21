import numpy as np
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.cereal.visionipc import VisionStreamType
from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.selfdrive.modeld.fill_model_msg import fill_pose_msg as fill_pose_msg_current
from openpilot.selfdrive.modeld.mi8_camera import compose_wide_camera_extrinsics, get_live_wide_camera_extrinsics, select_road_streams
from openpilot.sunnypilot.modeld_v2.fill_model_msg import fill_pose_msg as fill_pose_msg_v2


class Calibration:
  def __init__(self, wide_from_device_euler):
    self.wideFromDeviceEuler = wide_from_device_euler


def test_mi8_narrow_and_wide_pair():
  streams = [VisionStreamType.VISION_STREAM_NARROW_ROAD, VisionStreamType.VISION_STREAM_WIDE_ROAD]
  main, use_extra, main_wide, mi8_dual = select_road_streams(streams, mi8_hardware=True)
  assert main == VisionStreamType.VISION_STREAM_NARROW_ROAD
  assert use_extra and not main_wide and mi8_dual


def test_current_narrow_and_wide_pair_is_unchanged():
  streams = [VisionStreamType.VISION_STREAM_NARROW_ROAD, VisionStreamType.VISION_STREAM_WIDE_ROAD]
  main, use_extra, main_wide, mi8_dual = select_road_streams(streams, mi8_hardware=False)
  assert main == VisionStreamType.VISION_STREAM_NARROW_ROAD
  assert use_extra and not main_wide and not mi8_dual


def test_wide_only_fallback_is_unchanged():
  main, use_extra, main_wide, mi8_dual = select_road_streams([VisionStreamType.VISION_STREAM_WIDE_ROAD], mi8_hardware=True)
  assert main == VisionStreamType.VISION_STREAM_WIDE_ROAD
  assert not use_extra and main_wide and not mi8_dual


def test_live_wide_camera_extrinsics_validation():
  values = [0.01, -0.02, 0.03]
  np.testing.assert_allclose(get_live_wide_camera_extrinsics(Calibration(values)), values)
  assert get_live_wide_camera_extrinsics(Calibration([])) is None
  assert get_live_wide_camera_extrinsics(Calibration([0.0, 0.0, np.nan])) is None
  assert get_live_wide_camera_extrinsics(Calibration([0.0, 0.0, np.radians(5.1)])) is None


def test_wide_camera_residual_is_composed_with_live_calibration():
  current = np.radians([0.3, 1.2, -0.7])
  residual = np.radians([-0.1, 0.2, 0.15])
  composed = compose_wide_camera_extrinsics(current, residual)
  np.testing.assert_allclose(rot_from_euler(composed), rot_from_euler(current) @ rot_from_euler(residual), atol=1e-7)
  np.testing.assert_allclose(compose_wide_camera_extrinsics(None, residual), residual)


@pytest.mark.parametrize("fill_pose_msg", [fill_pose_msg_current, fill_pose_msg_v2])
def test_camera_odometry_publishes_absolute_wide_calibration(fill_pose_msg):
  current = np.radians([0.3, 1.2, -0.7])
  residual = np.radians([-0.1, 0.2, 0.15])
  outputs = {
    'pose': np.zeros((1, 6)),
    'pose_stds': np.ones((1, 6)),
    'wide_from_device_euler': residual.reshape(1, 3),
    'wide_from_device_euler_stds': np.ones((1, 3)),
    'road_transform': np.zeros((1, 6)),
    'road_transform_stds': np.ones((1, 6)),
  }

  msg = messaging.new_message('cameraOdometry')
  fill_pose_msg(msg, outputs, 1, 0, 2, True, current)
  actual = np.asarray(msg.cameraOdometry.wideFromDeviceEuler)
  expected = compose_wide_camera_extrinsics(current, residual)
  np.testing.assert_allclose(rot_from_euler(actual), rot_from_euler(expected), atol=1e-7)

  msg = messaging.new_message('cameraOdometry')
  fill_pose_msg(msg, outputs, 1, 0, 2, True)
  np.testing.assert_allclose(msg.cameraOdometry.wideFromDeviceEuler, residual)
