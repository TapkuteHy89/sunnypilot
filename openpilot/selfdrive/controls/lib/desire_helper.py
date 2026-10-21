from openpilot.cereal import log, custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeController, AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LaneTurnController

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = custom.ModelDataV2SP.TurnDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}

ATC_TURN_TYPES = ("turn left", "turn right")
ATC_LANE_CHANGE_TYPES = ("fork left", "fork right", "atc left", "atc right")
ATC_TYPES = ATC_TURN_TYPES + ATC_LANE_CHANGE_TYPES
VIRTUAL_BLINKER_NONE = 0
VIRTUAL_BLINKER_LEFT = 1
VIRTUAL_BLINKER_RIGHT = 2

class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none
    self.alc = AutoLaneChangeController(self)
    self.lane_turn_controller = LaneTurnController(self)
    self.lane_turn_direction = TurnDirection.none
    self.carrot_atc_active = False
    self.carrot_cmd_index_last = 0
    self.carrot_virtual_blinker = VIRTUAL_BLINKER_NONE
    self.carrot_lane_change_count = 0
    self.carrot_atc_type = ""
    self.carrot_atc_conflict = False

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def _get_effective_direction(self, carstate):
    """Give a physical one-sided blinker priority over navigation."""
    if carstate.leftBlinker != carstate.rightBlinker:
      return self.get_lane_change_direction(carstate)
    if self.carrot_virtual_blinker == VIRTUAL_BLINKER_LEFT:
      return LaneChangeDirection.left
    if self.carrot_virtual_blinker == VIRTUAL_BLINKER_RIGHT:
      return LaneChangeDirection.right
    return self.get_lane_change_direction(carstate)

  def _reset_carrot_atc(self):
    self.carrot_atc_active = False
    self.carrot_virtual_blinker = VIRTUAL_BLINKER_NONE
    self.carrot_lane_change_count = 0
    self.carrot_atc_type = ""
    self.carrot_atc_conflict = False

  def _update_carrot_atc(self, carrot_man, carstate, physical_blinker):
    """Translate guarded Carrot navigation/voice requests into a virtual blinker."""
    if carrot_man is None:
      self._reset_carrot_atc()
      return False

    self.carrot_lane_change_count = max(0, self.carrot_lane_change_count - 1)
    atc_type = str(carrot_man.atcType)
    type_changed = atc_type != self.carrot_atc_type

    if carrot_man.carrotCmdIndex != self.carrot_cmd_index_last:
      self.carrot_cmd_index_last = carrot_man.carrotCmdIndex
      if carrot_man.carrotCmd in ("LANECHANGE", "OVERTAKE") and carrot_man.carrotArg in ("LEFT", "RIGHT"):
        self.carrot_virtual_blinker = VIRTUAL_BLINKER_LEFT if carrot_man.carrotArg == "LEFT" else VIRTUAL_BLINKER_RIGHT
        self.carrot_lane_change_count = max(1, int(0.2 / DT_MDL))

    if self.carrot_lane_change_count > 0:
      virtual_blinker = self.carrot_virtual_blinker
    elif atc_type in ATC_TYPES:
      virtual_blinker = VIRTUAL_BLINKER_LEFT if "left" in atc_type else VIRTUAL_BLINKER_RIGHT
    else:
      virtual_blinker = VIRTUAL_BLINKER_NONE
      self.carrot_atc_conflict = False

    driver_opposes_atc = (physical_blinker != VIRTUAL_BLINKER_NONE and
                          virtual_blinker != VIRTUAL_BLINKER_NONE and
                          physical_blinker != virtual_blinker)
    steering_opposes_atc = (
      virtual_blinker == VIRTUAL_BLINKER_LEFT and carstate.steeringPressed and carstate.steeringTorque < 0
    ) or (
      virtual_blinker == VIRTUAL_BLINKER_RIGHT and carstate.steeringPressed and carstate.steeringTorque > 0
    )
    if driver_opposes_atc or steering_opposes_atc:
      self.carrot_atc_conflict = True

    # Match the author's path: ignore the first model frame of a route-type transition.
    route_transition = type_changed and atc_type in ATC_TYPES and self.carrot_lane_change_count == 0
    if self.carrot_atc_conflict or route_transition:
      virtual_blinker = VIRTUAL_BLINKER_NONE

    self.carrot_atc_type = atc_type
    self.carrot_virtual_blinker = virtual_blinker
    self.carrot_atc_active = virtual_blinker != VIRTUAL_BLINKER_NONE
    return self.carrot_atc_active and atc_type in ATC_TURN_TYPES

  def update(self, carstate, lateral_active, lane_change_prob, carrot_man=None,
             left_edge_detected=False, right_edge_detected=False):
    self.alc.update_params()
    self.lane_turn_controller.update_params()
    v_ego = carstate.vEgo
    physical_one_blinker = carstate.leftBlinker != carstate.rightBlinker
    physical_blinker = VIRTUAL_BLINKER_NONE
    if physical_one_blinker:
      physical_blinker = VIRTUAL_BLINKER_LEFT if carstate.leftBlinker else VIRTUAL_BLINKER_RIGHT
    atc_turn_active = self._update_carrot_atc(carrot_man, carstate, physical_blinker)
    one_blinker = physical_one_blinker or self.carrot_atc_active
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN
    if atc_turn_active:
      below_lane_change_speed = True

    # Lane turn controller update
    self.lane_turn_controller.update_lane_turn(blindspot_left=carstate.leftBlindspot, blindspot_right=carstate.rightBlindspot,
                                               left_blinker=carstate.leftBlinker, right_blinker=carstate.rightBlinker, v_ego=v_ego)
    if not lateral_active:
      self.lane_turn_direction = TurnDirection.none
    elif atc_turn_active:
      self.lane_turn_direction = (TurnDirection.turnLeft if self.carrot_virtual_blinker == VIRTUAL_BLINKER_LEFT
                                  else TurnDirection.turnRight)
    else:
      self.lane_turn_direction = self.lane_turn_controller.get_turn_direction()

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or self.alc.lane_change_set_timer == AutoLaneChangeMode.OFF:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
    else:
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self._get_effective_direction(carstate)

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Update lane change direction
        self.lane_change_direction = self._get_effective_direction(carstate)

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = (((carstate.leftBlindspot or left_edge_detected) and self.lane_change_direction == LaneChangeDirection.left) or
                              ((carstate.rightBlindspot or right_edge_detected) and self.lane_change_direction == LaneChangeDirection.right))

        self.alc.update_lane_change(blindspot_detected, carstate.brakePressed)

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
        elif (torque_applied or self.alc.auto_lane_change_allowed) and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        if lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          self.lane_change_timer = 0.0
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_direction = self._get_effective_direction(carstate)
          else:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none

    self.prev_one_blinker = one_blinker and lateral_active

    if self.lane_turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.lane_turn_direction]
    else:
      self.desire = log.Desire.none
      if self.lane_change_state == LaneChangeState.laneChangeStarting:
        if self.lane_change_direction == LaneChangeDirection.left:
          self.desire = log.Desire.laneChangeLeft
        elif self.lane_change_direction == LaneChangeDirection.right:
          self.desire = log.Desire.laneChangeRight

    self.alc.update_state()
