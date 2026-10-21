"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs, apply_hysteresis
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER, update_manual_button_timers

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState

ALLOWED_SPEED_THRESHOLD = 1.8  # m/s, ~4 MPH
HYST_GAP = 0.0  # currently disabled; TODO-SP: might need to be brand-specific
INACTIVE_TIMER = 0.4
MIN_VALID_TARGET = 1.0
# Route 10d briefly lost Vision curve state for 1.94 s, then re-entered the
# same turn. Keep the latched curve target through that observed blind spot.
CURVE_EXIT_CONFIRM_TIME = 2.25
CURVE_EXIT_CONFIRM_FRAMES = max(1, round(CURVE_EXIT_CONFIRM_TIME / DT_CTRL))

CURVE_SOURCES = (
  LongitudinalPlanSource.sccVision,
  LongitudinalPlanSource.sccMap,
)


SEND_BUTTONS = {
  State.increasing: SendButtonState.increase,
  State.decreasing: SendButtonState.decrease,
}


class IntelligentCruiseButtonManagement:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.v_target = 0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.cruise_button = SendButtonState.none
    self.state = State.inactive
    self.pre_active_timer = 0

    self.is_ready = False
    self.is_ready_prev = False
    self.v_target_ms_last = 0.0
    self.is_metric = False

    self.cruise_button_timers = CRUISE_BUTTON_TIMER
    self.curve_cycle_active = False
    self.restore_speed_ms = 0.0
    self.curve_min_target_ms = 0.0
    self.curve_exit_counter = 0
    self.curve_exit_confirmed = False
    self.target_command_valid = False

  @property
  def v_cruise_equal(self) -> bool:
    return self.v_target == self.v_cruise_cluster

  def update_calculations(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    ms_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    current_setpoint_ms = CS.cruiseState.speedCluster
    source = LP_SP.longitudinalPlanSource
    plan_target_ms = float(LP_SP.vTarget)
    plan_target_valid = math.isfinite(plan_target_ms) and plan_target_ms >= MIN_VALID_TARGET
    curve_target_selected = source in CURVE_SOURCES and plan_target_valid
    scc = LP_SP.smartCruiseControl
    curve_control_active = bool(scc.vision.active or scc.map.active)

    if not self.curve_cycle_active and curve_target_selected and plan_target_ms < current_setpoint_ms - 0.5 * ms_conv:
      self.curve_cycle_active = True
      self.restore_speed_ms = current_setpoint_ms
      self.curve_min_target_ms = plan_target_ms
      self.curve_exit_counter = 0
      self.curve_exit_confirmed = False

    if self.curve_cycle_active:
      if curve_target_selected:
        self.curve_min_target_ms = min(self.curve_min_target_ms, plan_target_ms)

      # The selected longitudinal source can briefly return to cruise while
      # Vision or Map is still in a turn state. Retain the lowest target and
      # require both controllers to stay clear for a full confirmation window
      # before any RES/+ command is allowed.
      if curve_control_active or curve_target_selected:
        self.curve_exit_counter = 0
        self.curve_exit_confirmed = False
      elif not self.curve_exit_confirmed:
        self.curve_exit_counter += 1
        self.curve_exit_confirmed = self.curve_exit_counter >= CURVE_EXIT_CONFIRM_FRAMES

      if self.curve_exit_confirmed:
        command_target_ms = self.restore_speed_ms
        if current_setpoint_ms >= self.restore_speed_ms - 0.5 * ms_conv:
          self.abort_cycle()
          command_target_ms = current_setpoint_ms
      else:
        # A 5 km/h OEM SET/- step can undershoot the continuously calculated
        # target. Never counter it with RES/+ before curve exit is confirmed.
        command_target_ms = min(self.curve_min_target_ms, current_setpoint_ms)

      self.target_command_valid = self.curve_cycle_active
    else:
      command_target_ms = current_setpoint_ms
      self.target_command_valid = False

    self.v_target_ms_last = apply_hysteresis(command_target_ms, self.v_target_ms_last, HYST_GAP * ms_conv)

    self.v_target = round(self.v_target_ms_last * speed_conv)
    self.v_cruise_min = get_minimum_set_speed(self.is_metric)
    self.v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)

  def update_state_machine(self) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)

    # HOLDING, ACCELERATING, DECELERATING, PRE_ACTIVE
    if self.state != State.inactive:
      if not self.is_ready:
        self.state = State.inactive

      else:
        # PRE_ACTIVE
        if self.state == State.preActive:
          if self.pre_active_timer <= 0:
            if self.v_cruise_equal:
              self.state = State.holding

            elif self.v_target > self.v_cruise_cluster:
              self.state = State.increasing

            elif self.v_target < self.v_cruise_cluster and self.v_cruise_cluster > self.v_cruise_min:
              self.state = State.decreasing

        # HOLDING
        elif self.state == State.holding:
          if not self.v_cruise_equal:
            self.state = State.preActive

        # ACCELERATING
        elif self.state == State.increasing:
          if self.v_target <= self.v_cruise_cluster:
            self.state = State.holding

        # DECELERATING
        elif self.state == State.decreasing:
          if self.v_target >= self.v_cruise_cluster or self.v_cruise_cluster <= self.v_cruise_min:
            self.state = State.holding

    # INACTIVE
    elif self.state == State.inactive:
      if self.is_ready and not self.is_ready_prev:
        self.pre_active_timer = int(INACTIVE_TIMER / DT_CTRL)
        self.state = State.preActive

    send_button = SEND_BUTTONS.get(self.state, SendButtonState.none)

    return send_button

  def abort_cycle(self) -> None:
    self.curve_cycle_active = False
    self.restore_speed_ms = 0.0
    self.curve_min_target_ms = 0.0
    self.curve_exit_counter = 0
    self.curve_exit_confirmed = False
    self.target_command_valid = False

  def update_readiness(self, CS: car.CarState, CC: car.CarControl, plan_valid: bool, force_decel: bool) -> None:
    update_manual_button_timers(CS, self.cruise_button_timers)

    ready = (CC.enabled and CC.longActive and plan_valid and not force_decel and self.target_command_valid and
             not CC.cruiseControl.override and not CC.cruiseControl.cancel and not CC.cruiseControl.resume)
    button_pressed = any(self.cruise_button_timers[k] > 0 for k in self.cruise_button_timers)

    if button_pressed or not CC.longActive or not plan_valid or force_decel or CC.cruiseControl.cancel:
      self.abort_cycle()

    self.is_ready = ready and not button_pressed

  def run(self, CS: car.CarState, CC: car.CarControl, LP_SP: custom.LongitudinalPlanSP, is_metric: bool,
          plan_valid: bool = True, force_decel: bool = False) -> None:
    if self.CP_SP.pcmCruiseSpeed:
      return

    self.is_metric = is_metric

    self.update_calculations(CS, LP_SP)
    self.update_readiness(CS, CC, plan_valid, force_decel)

    self.cruise_button = self.update_state_machine()

    self.is_ready_prev = self.is_ready
