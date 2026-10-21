from openpilot.common.test import OpenpilotTestCase
from openpilot.common.parameterized import parameterized

from openpilot.cereal import log
from opendbc.car.structs import car
from opendbc.car.car_helpers import interfaces
from opendbc.car.baic.values import CAR as BAIC
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.latcontrol_torque import (
  BAIC_STRAIGHT_KP_SCALE,
  LAT_ACCEL_REQUEST_BUFFER_SECONDS,
  LatControlTorque,
  lateral_error_scale,
)

from openpilot.selfdrive.car.helpers import convert_to_capnp
from openpilot.selfdrive.locationd.helpers import Pose
from openpilot.common.mock.generators import generate_deviceMotion
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0


def get_controller_components(car_name):
  CarInterface = interfaces[car_name]
  CP = CarInterface.get_non_essential_params(car_name)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
  CI = CarInterface(CP, CP_SP)
  sunnypilot_interfaces.setup_interfaces(CI)
  CP_SP = convert_to_capnp(CP_SP)
  VM = VehicleModel(CP)
  return CP, CP_SP, CI, VM


def get_controller(car_name):
  CP, CP_SP, CI, VM = get_controller_components(car_name)
  return LatControlTorque(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL), VM


class FakeParams:
  def __init__(self, enforce: bool, version: float):
    self.enforce = enforce
    self.version = version

  def get_bool(self, key):
    assert key == "EnforceTorqueControl"
    return self.enforce

  def get(self, key):
    assert key == "TorqueControlTune"
    return self.version

class TestLatControlTorqueBuffer(OpenpilotTestCase):

  def test_baic_small_signal_gain_blends_to_full_curve_gain(self):
    assert lateral_error_scale(False, 0.0, 0.0) == 1.0
    assert lateral_error_scale(True, 0.1, -0.2) == BAIC_STRAIGHT_KP_SCALE
    assert BAIC_STRAIGHT_KP_SCALE < lateral_error_scale(True, 0.5, 0.1) < 1.0
    assert lateral_error_scale(True, 1.0, 0.0) == 1.0

  def test_baic_runtime_controller_versions_share_small_signal_gain(self):
    CP, CP_SP, CI, _ = get_controller_components(BAIC.BEIJING_X7_2022)
    v1 = LatControlTorque(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)
    controls_ext = object.__new__(ControlsExt)
    controls_ext.CP = CP.as_reader()
    controls_ext.CP_SP = CP_SP.as_reader()

    controls_ext.params = FakeParams(False, 0.0)
    selected = controls_ext.initialize_lateral_control(v1, CI, DT_CTRL)
    assert isinstance(selected, LatControlTorqueV0)
    assert selected.is_baic_x7
    assert lateral_error_scale(selected.is_baic_x7, 0.1, -0.2) == BAIC_STRAIGHT_KP_SCALE

    controls_ext.params = FakeParams(True, 1.0)
    assert controls_ext.initialize_lateral_control(v1, CI, DT_CTRL) is v1
    assert v1.is_baic_x7

  @parameterized.expand([(TOYOTA.TOYOTA_COROLLA_TSS2,)])
  def test_request_buffer_consistency(self, car_name):
    buffer_steps = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / DT_CTRL)
    controller, VM = get_controller(car_name)

    CS = car.CarState.new_message()
    CS.vEgo = 30
    CS.steeringPressed = False
    params = log.VehicleParameters.new_message()

    lp = generate_deviceMotion()
    pose = Pose.from_device_motion(lp.deviceMotion)

    for _ in range(buffer_steps):
      controller.update(True, CS, VM, params, False, 0.001, pose, False, 0.2)
    assert all(val != 0 for val in controller.lat_accel_request_buffer)

    for _ in range(buffer_steps):
      controller.update(False, CS, VM, params, False, 0.0, pose, False, 0.2)
    assert all(val == 0 for val in controller.lat_accel_request_buffer)
