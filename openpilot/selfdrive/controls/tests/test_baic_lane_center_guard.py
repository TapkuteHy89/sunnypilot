from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.selfdrive.controls.controlsd import Controls


def line(y0: float, y10: float | None = None, y20: float | None = None):
  return SimpleNamespace(x=[0.0, 10.0, 20.0], y=[y0, y0 if y10 is None else y10, y0 if y20 is None else y20])


def model(left, right, path, probs=(0.95, 0.95), lane_change=log.LaneChangeState.off):
  return SimpleNamespace(
    laneLines=[line(-5.0), left, right],
    laneLineProbs=[0.0, probs[0], probs[1]],
    position=path,
    meta=SimpleNamespace(laneChangeState=lane_change),
  )


def new_guard():
  guard = Controls.__new__(Controls)
  guard.x7_lane_center_guard = True
  guard.x7_lane_center_guard_lat_accel = 0.0
  guard.x7_lane_center_guard_apply = False
  return guard


def car_state(speed=12.0, steering_pressed=False):
  return SimpleNamespace(
    vEgo=speed, steeringPressed=steering_pressed, leftBlinker=False, rightBlinker=False,
  )


def test_centered_path_has_no_correction():
  guard = new_guard()
  guard._update_x7_lane_center_guard(model(line(-1.6), line(1.6), line(0.0)), car_state(), True)
  assert guard.x7_lane_center_guard_apply
  assert guard.x7_lane_center_guard_lat_accel == 0.0


def test_left_bias_corrects_right_and_right_bias_corrects_left():
  left_biased = new_guard()
  left_biased._update_x7_lane_center_guard(model(line(-1.1), line(2.1), line(0.0)), car_state(), True)
  assert left_biased.x7_lane_center_guard_lat_accel > 0.0

  right_biased = new_guard()
  right_biased._update_x7_lane_center_guard(model(line(-2.1), line(1.1), line(0.0)), car_state(), True)
  assert right_biased.x7_lane_center_guard_lat_accel < 0.0


def test_guard_releases_for_low_confidence_driver_input_and_lane_change():
  cases = (
    (model(line(-1.1), line(2.1), line(0.0), probs=(0.59, 0.95)), car_state(), log.LaneChangeState.off),
    (model(line(-1.1), line(2.1), line(0.0)), car_state(steering_pressed=True), log.LaneChangeState.off),
    (model(line(-1.1), line(2.1), line(0.0), lane_change=log.LaneChangeState.preLaneChange), car_state(),
     log.LaneChangeState.preLaneChange),
  )
  for candidate, state, _ in cases:
    guard = new_guard()
    guard.x7_lane_center_guard_lat_accel = 0.1
    guard._update_x7_lane_center_guard(candidate, state, True)
    assert not guard.x7_lane_center_guard_apply
    assert 0.0 < guard.x7_lane_center_guard_lat_accel < 0.1


def test_guard_correction_is_bounded():
  guard = new_guard()
  for _ in range(100):
    guard._update_x7_lane_center_guard(model(line(0.0), line(3.2), line(-3.0)), car_state(), True)
  from openpilot.selfdrive.controls.controlsd import X7_LANE_CENTER_MAX_LAT_ACCEL
  assert 0.0 < guard.x7_lane_center_guard_lat_accel <= X7_LANE_CENTER_MAX_LAT_ACCEL
