from openpilot.cereal.visionipc import VisionStreamType
from openpilot.selfdrive.modeld.mi8_camera import select_road_streams


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
