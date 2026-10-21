from opendbc.car.structs import car
import numpy as np

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.locationd.torqued import BAIC_LAT_ACCEL_OFFSET_LIMIT, MIN_BUCKET_POINTS, TorqueEstimator
from openpilot.sunnypilot.selfdrive.locationd.torqued_ext import RELAXED_MIN_BUCKET_POINTS


class TestTorqued(OpenpilotTestCase):
  def test_baic_live_params_and_offset_limit(self):
    Params().put_bool("LiveTorqueParamsRelaxedToggle", True)
    CP = car.CarParams.new_message()
    CP.brand = 'baic'
    CP.lateralTuning.init('torque')
    CP.lateralTuning.torque.latAccelFactor = 0.55
    CP.lateralTuning.torque.friction = 0.1
    est = TorqueEstimator(CP)
    assert est.use_params
    assert est.lat_accel_offset_limit == BAIC_LAT_ACCEL_OFFSET_LIMIT
    np.testing.assert_array_equal(est.min_bucket_points, RELAXED_MIN_BUCKET_POINTS)
    assert est.factor_sanity == 0.3
    assert est.friction_sanity == 0.5

  def test_relaxed_sampling_does_not_apply_to_other_brands_without_enforce(self):
    Params().put_bool("LiveTorqueParamsRelaxedToggle", True)
    CP = car.CarParams.new_message()
    CP.brand = 'toyota'
    CP.lateralTuning.init('torque')
    CP.lateralTuning.torque.latAccelFactor = 0.55
    CP.lateralTuning.torque.friction = 0.1
    est = TorqueEstimator(CP)
    np.testing.assert_array_equal(est.min_bucket_points, MIN_BUCKET_POINTS)

  def test_cal_percent(self):
    est = TorqueEstimator(car.CarParams())
    msg = est.get_msg()
    assert msg.lateralTorqueParameters.calPerc == 0

    for (low, high), min_pts in zip(est.filtered_points.buckets.keys(),
                                    est.filtered_points.buckets_min_points.values(), strict=True):
      for _ in range(int(min_pts)):
        est.filtered_points.add_point((low + high) / 2.0, 0.0)

    # enough bucket points, but not enough total points
    msg = est.get_msg()
    assert msg.lateralTorqueParameters.calPerc == (len(est.filtered_points) / est.min_points_total * 100 + 100) / 2

    # add enough points to bucket with most capacity
    key = list(est.filtered_points.buckets)[0]
    for _ in range(est.min_points_total - len(est.filtered_points)):
      est.filtered_points.add_point((key[0] + key[1]) / 2.0, 0.0)

    msg = est.get_msg()
    assert msg.lateralTorqueParameters.calPerc == 100
