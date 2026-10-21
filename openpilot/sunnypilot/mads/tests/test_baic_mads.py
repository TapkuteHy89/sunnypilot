import unittest
from unittest.mock import MagicMock, patch

from openpilot.cereal import custom, log
from opendbc.car import structs
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt


State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
ButtonType = structs.CarState.ButtonEvent.Type


def make_car_state(available: bool, enabled: bool, button_events=None, v_ego: float = 0.0, brake: bool = False) -> structs.CarState:
  return structs.CarState(
    cruiseState={"available": available, "enabled": enabled},
    buttonEvents=[] if button_events is None else button_events,
    vEgo=v_ego,
    brakePressed=brake,
  )


def make_mads(brand: str = "baic", openpilot_longitudinal: bool = False,
              main_cruise_allowed: bool = True, unified_engagement: bool = False):
  params = MagicMock()
  values = {
    "Mads": True,
    "MadsMainCruiseAllowed": main_cruise_allowed,
    "MadsUnifiedEngagementMode": unified_engagement,
    "DisengageOnAccelerator": True,
  }
  params.get_bool.side_effect = lambda key: values.get(key, False)
  params.get.return_value = MadsSteeringModeOnBrake.REMAIN_ACTIVE

  selfdrive = MagicMock()
  selfdrive.CP = structs.CarParams(
    brand=brand,
    openpilotLongitudinalControl=openpilot_longitudinal,
    minSteerSpeed=1.0 / 3.6 if brand == "baic" else 0.0,
  )
  selfdrive.CP_SP = structs.CarParamsSP()
  selfdrive.params = params
  selfdrive.events = Events()
  selfdrive.events_sp = EventsSP()
  selfdrive.enabled = False
  selfdrive.enabled_prev = False
  selfdrive.initialized = True
  selfdrive.CS_prev = make_car_state(False, False)
  selfdrive.state_machine = MagicMock()
  panda_state = MagicMock()
  panda_state.safetyModel = structs.CarParams.SafetyModel.baicX7
  panda_state.controlsAllowedLateral = True
  selfdrive.sm = {"pandaStates": [panda_state]}

  with patch("openpilot.sunnypilot.mads.mads.Params", return_value=params):
    mads = ModularAssistiveDrivingSystem(selfdrive)

  return mads, selfdrive


def clear_events(selfdrive) -> None:
  selfdrive.events.clear()
  selfdrive.events_sp.clear()


class TestBaicMadsSession(unittest.TestCase):
  def test_unified_acc_enable_establishes_mads(self):
    mads, selfdrive = make_mads(unified_engagement=True)
    selfdrive.events.add(EventName.pcmEnable)

    mads.update(make_car_state(True, True))

    self.assertEqual(mads.state_machine.state, State.enabled)
    self.assertTrue(mads.enabled)
    self.assertTrue(mads.active)

  def test_acc_event_does_not_force_mads_when_unified_is_disabled(self):
    mads, selfdrive = make_mads()
    selfdrive.events.add(EventName.pcmEnable)

    mads.update(make_car_state(True, True))

    self.assertEqual(mads.state_machine.state, State.disabled)
    self.assertFalse(mads.active)
    self.assertFalse(selfdrive.events.has(EventName.pcmEnable))

  def test_baic_acc_available_stage_does_not_enable_lateral(self):
    mads, selfdrive = make_mads()
    selfdrive.CS_prev = make_car_state(False, False)
    standby = make_car_state(True, False)
    mads.update(standby)
    self.assertFalse(mads.active)
    self.assertFalse(selfdrive.events_sp.has(EventNameSP.lkasEnable))

  def test_brake_acc_falling_edge_and_release_keep_lateral(self):
    mads, selfdrive = make_mads(unified_engagement=True)
    mads.state_machine.state = State.enabled
    mads.enabled = True
    mads.active = True
    active = make_car_state(True, True)
    selfdrive.CS_prev = active

    clear_events(selfdrive)
    selfdrive.events.add(EventName.pedalPressed)
    standby = make_car_state(True, False, v_ego=10.0, brake=True)
    mads.update(standby)
    self.assertTrue(mads.active)
    self.assertFalse(selfdrive.events_sp.has(EventNameSP.lkasDisable))
    controls = MagicMock()
    controls.blinker_pause_lateral.update.return_value = False
    ss_sp = custom.SelfdriveStateSP.new_message()
    ss_sp.mads.available = True
    ss_sp.mads.active = mads.active
    sm = {"carState": standby, "selfdriveStateSP": ss_sp}
    self.assertTrue(ControlsExt.get_lat_active(controls, sm))

    clear_events(selfdrive)
    selfdrive.CS_prev = standby
    mads.update(make_car_state(True, False, v_ego=10.0))
    self.assertTrue(mads.active)
    self.assertFalse(selfdrive.enabled)
    self.assertFalse(selfdrive.events.has(EventName.pcmEnable))

    clear_events(selfdrive)
    selfdrive.CS_prev = make_car_state(True, False, v_ego=10.0)
    mads.update(make_car_state(False, False, v_ego=10.0))
    self.assertFalse(mads.active)
    self.assertTrue(selfdrive.events_sp.has(EventNameSP.lkasDisable))

  def test_brake_pause_and_disengage_modes_still_apply(self):
    for brake_mode, expected_state in ((MadsSteeringModeOnBrake.PAUSE, State.paused),
                                       (MadsSteeringModeOnBrake.DISENGAGE, State.disabled)):
      with self.subTest(brake_mode=brake_mode):
        mads, selfdrive = make_mads(unified_engagement=True)
        mads.steering_mode_on_brake = brake_mode
        mads.state_machine.state = State.enabled
        mads.enabled = mads.active = True
        selfdrive.CS_prev = make_car_state(True, True, v_ego=10.0)
        selfdrive.events.add(EventName.pedalPressed)
        mads.update(make_car_state(True, False, v_ego=10.0, brake=True))
        self.assertFalse(mads.active)
        self.assertEqual(mads.state_machine.state, expected_state)

  def test_sustained_panda_lateral_denial_still_disengages(self):
    mads, selfdrive = make_mads()
    mads.state_machine.state = State.enabled
    mads.enabled = mads.active = True
    selfdrive.sm["pandaStates"][0].controlsAllowedLateral = False
    driving = make_car_state(True, False, v_ego=10.0, brake=True)
    for _ in range(200):
      clear_events(selfdrive)
      mads.update(driving)
      selfdrive.CS_prev = driving
    self.assertFalse(mads.active)
    self.assertTrue(selfdrive.events_sp.has(EventNameSP.controlsMismatchLateral))

  def test_non_baic_keeps_upstream_cruise_availability_behavior(self):
    mads, selfdrive = make_mads(brand="hyundai", openpilot_longitudinal=False)
    selfdrive.CS_prev = make_car_state(False, False)
    standby = make_car_state(True, False)
    mads.update(standby)

    self.assertTrue(mads.active)
    self.assertTrue(selfdrive.events_sp.has(EventNameSP.lkasEnable))

    clear_events(selfdrive)
    selfdrive.CS_prev = standby
    unavailable = make_car_state(False, False)
    mads.update(unavailable)
    self.assertFalse(mads.active)
    self.assertTrue(selfdrive.events_sp.has(EventNameSP.lkasDisable))

  def test_baic_lka_press_is_ignored(self):
    mads, selfdrive = make_mads(main_cruise_allowed=False)
    selfdrive.CS_prev = make_car_state(True, False)
    lka_press = [{"type": ButtonType.lkas, "pressed": True}]

    mads.update(make_car_state(True, False, lka_press))

    self.assertEqual(mads.state_machine.state, State.disabled)
    self.assertFalse(mads.enabled)
    self.assertFalse(mads.active)
    self.assertFalse(selfdrive.events_sp.has(EventNameSP.lkasEnable))

  def test_lka_off_edge_cannot_reenable_after_safety_disengagement(self):
    mads, selfdrive = make_mads(main_cruise_allowed=False)
    selfdrive.CS_prev = make_car_state(True, False)

    mads.update(make_car_state(True, False, [{"type": ButtonType.lkas, "pressed": False}]))

    self.assertEqual(mads.state_machine.state, State.disabled)
    self.assertFalse(mads.enabled)
    self.assertFalse(mads.active)
    self.assertFalse(selfdrive.events_sp.has(EventNameSP.lkasEnable))

  def test_baic_lka_edges_never_change_mads(self):
    mads, selfdrive = make_mads(main_cruise_allowed=False)
    selfdrive.CS_prev = make_car_state(True, False)

    mads.update(make_car_state(True, False, [{"type": ButtonType.lkas, "pressed": True}]))
    self.assertFalse(mads.active)

    clear_events(selfdrive)
    selfdrive.CS_prev = make_car_state(True, False)
    mads.update(make_car_state(True, False, [{"type": ButtonType.lkas, "pressed": False}]))
    self.assertFalse(mads.active)
    self.assertFalse(selfdrive.events_sp.has(EventNameSP.lkasDisable))

  def test_all_brands_follow_same_cruise_availability_path(self):
    mads, selfdrive = make_mads(brand="hyundai", openpilot_longitudinal=False)
    mads.state_machine.state = State.enabled
    mads.enabled = True
    mads.active = True
    selfdrive.CS_prev = make_car_state(True, True)

    mads.update(make_car_state(False, False))

    self.assertEqual(mads.state_machine.state, State.disabled)
    self.assertTrue(selfdrive.events_sp.has(EventNameSP.lkasDisable))

  def test_transient_lateral_mismatch_counter_resets_on_agreement(self):
    mads, selfdrive = make_mads()
    mads.state_machine.state = State.enabled
    mads.enabled = True
    mads.active = True
    panda_state = selfdrive.sm["pandaStates"][0]
    panda_state.controlsAllowedLateral = False

    driving = make_car_state(True, False, v_ego=10.0)
    for _ in range(100):
      mads.data_sample(driving)
    self.assertEqual(mads.lateral_mismatch_counter, 100)

    panda_state.controlsAllowedLateral = True
    mads.data_sample(driving)
    self.assertEqual(mads.lateral_mismatch_counter, 0)

  def test_standstill_lateral_arm_is_not_a_safety_mismatch(self):
    mads, selfdrive = make_mads()
    mads.state_machine.state = State.enabled
    mads.enabled = True
    mads.active = True
    selfdrive.sm["pandaStates"][0].controlsAllowedLateral = False

    for _ in range(250):
      mads.data_sample(make_car_state(True, False, v_ego=0.0))

    self.assertEqual(mads.lateral_mismatch_counter, 0)


if __name__ == "__main__":
  unittest.main()
