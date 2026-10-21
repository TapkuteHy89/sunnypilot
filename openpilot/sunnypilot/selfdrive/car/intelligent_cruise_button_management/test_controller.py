from types import SimpleNamespace

from openpilot.cereal import custom
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import (
  CURVE_EXIT_CONFIRM_FRAMES,
  IntelligentCruiseButtonManagement,
  SendButtonState,
  State,
)


LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource


def make_controller():
  return IntelligentCruiseButtonManagement(
    SimpleNamespace(),
    SimpleNamespace(pcmCruiseSpeed=False),
  )


def make_state(setpoint_ms=20.0):
  return SimpleNamespace(
    cruiseState=SimpleNamespace(speedCluster=setpoint_ms),
    buttonEvents=[],
  )


def make_control(long_active=True):
  return SimpleNamespace(
    enabled=True,
    longActive=long_active,
    cruiseControl=SimpleNamespace(override=False, cancel=False, resume=False),
  )


def make_plan(source, target_ms, vision_active=False, map_active=False):
  return SimpleNamespace(
    longitudinalPlanSource=source,
    vTarget=target_ms,
    smartCruiseControl=SimpleNamespace(
      vision=SimpleNamespace(active=vision_active),
      map=SimpleNamespace(active=map_active),
    ),
  )


def test_zero_force_decel_target_never_sends_set_minus():
  controller = make_controller()
  state = make_state()
  control = make_control()

  controller.run(state, control, make_plan(LongitudinalPlanSource.sccMap, 16.0), True)
  assert controller.curve_cycle_active
  assert controller.is_ready

  controller.run(state, control, make_plan(LongitudinalPlanSource.cruise, 0.0), True,
                 plan_valid=True, force_decel=True)
  assert not controller.curve_cycle_active
  assert not controller.is_ready
  assert controller.cruise_button == SendButtonState.none


def test_curve_cycle_holds_lowest_target_until_confirmed_exit_then_restores():
  controller = make_controller()
  control = make_control()

  controller.run(make_state(20.0), control,
                 make_plan(LongitudinalPlanSource.sccVision, 16.0, vision_active=True), True)
  assert controller.curve_cycle_active
  assert controller.restore_speed_ms == 20.0
  assert controller.curve_min_target_ms == 16.0
  assert controller.v_target == round(16.0 * 3.6)

  # A lower curve target is latched for the rest of the cycle.
  controller.run(make_state(16.0), control,
                 make_plan(LongitudinalPlanSource.sccVision, 15.0, vision_active=True), True)
  assert controller.curve_min_target_ms == 15.0

  # A transient source change to cruise while Vision remains active must not
  # restore the setpoint or send RES/+.
  controller.state = State.holding
  controller.is_ready_prev = True
  controller.run(make_state(15.0), control,
                 make_plan(LongitudinalPlanSource.cruise, 20.0, vision_active=True), True)
  assert controller.curve_cycle_active
  assert not controller.curve_exit_confirmed
  assert controller.target_command_valid
  assert controller.v_target == round(15.0 * 3.6)
  assert controller.cruise_button != SendButtonState.increase

  # Even after both controllers report clear, require a stable confirmation
  # window before permitting setpoint restoration.
  clear_plan = make_plan(LongitudinalPlanSource.cruise, 20.0)
  for _ in range(CURVE_EXIT_CONFIRM_FRAMES - 1):
    controller.run(make_state(15.0), control, clear_plan, True)
    assert controller.v_target == round(15.0 * 3.6)
    assert controller.cruise_button != SendButtonState.increase

  controller.run(make_state(15.0), control, clear_plan, True)
  assert controller.curve_exit_confirmed
  assert controller.v_target == round(20.0 * 3.6)

  # The ordinary state-machine pacing applies after exit confirmation.
  for _ in range(CURVE_EXIT_CONFIRM_FRAMES):
    controller.run(make_state(15.0), control, clear_plan, True)
    if controller.cruise_button == SendButtonState.increase:
      break
  assert controller.cruise_button == SendButtonState.increase

  controller.run(make_state(20.0), control, clear_plan, True)
  assert not controller.curve_cycle_active
  assert not controller.target_command_valid


def test_curve_setpoint_undershoot_never_causes_res_plus_before_exit():
  controller = make_controller()
  control = make_control()

  controller.run(make_state(50 / 3.6), control,
                 make_plan(LongitudinalPlanSource.sccVision, 42 / 3.6, vision_active=True), True)
  controller.state = State.holding
  controller.is_ready_prev = True

  # OEM SET/- changes the setpoint by 5 km/h and undershoots the held 42 km/h
  # target to 40 km/h. Keep 40 instead of reversing direction with RES/+.
  controller.run(make_state(40 / 3.6), control,
                 make_plan(LongitudinalPlanSource.sccVision, 48 / 3.6, vision_active=True), True)
  assert controller.curve_min_target_ms == 42 / 3.6
  assert controller.v_target == 40
  assert controller.cruise_button != SendButtonState.increase


def test_fragmented_curve_detection_does_not_restore_during_observed_gap():
  controller = make_controller()
  control = make_control()

  controller.run(make_state(50 / 3.6), control,
                 make_plan(LongitudinalPlanSource.sccVision, 40 / 3.6, vision_active=True), True)
  controller.state = State.holding
  controller.is_ready_prev = True

  # Route 10d lost the curve state for 1.94 s before Vision selected the same
  # curve again. The held target must survive the whole measured gap.
  clear_plan = make_plan(LongitudinalPlanSource.cruise, 50 / 3.6)
  observed_gap_frames = round(1.94 / 0.01)
  assert observed_gap_frames < CURVE_EXIT_CONFIRM_FRAMES
  for _ in range(observed_gap_frames):
    controller.run(make_state(40 / 3.6), control, clear_plan, True)
    assert not controller.curve_exit_confirmed
    assert controller.v_target == 40
    assert controller.cruise_button != SendButtonState.increase

  controller.run(make_state(40 / 3.6), control,
                 make_plan(LongitudinalPlanSource.sccVision, 39 / 3.6, vision_active=True), True)
  assert controller.curve_exit_counter == 0
  assert controller.curve_min_target_ms == 39 / 3.6
  assert controller.cruise_button != SendButtonState.increase


def test_requires_valid_plan_and_longitudinal_authority():
  controller = make_controller()
  state = make_state()

  controller.run(state, make_control(long_active=False),
                 make_plan(LongitudinalPlanSource.sccMap, 16.0), True)
  assert not controller.is_ready
  assert not controller.curve_cycle_active

  controller.run(state, make_control(), make_plan(LongitudinalPlanSource.sccMap, 16.0), True,
                 plan_valid=False)
  assert not controller.is_ready
  assert not controller.curve_cycle_active


def test_generic_cruise_target_cannot_start_button_control():
  controller = make_controller()
  controller.run(make_state(), make_control(), make_plan(LongitudinalPlanSource.cruise, 0.0), True)
  assert not controller.is_ready
  assert not controller.curve_cycle_active
  assert controller.cruise_button == SendButtonState.none
