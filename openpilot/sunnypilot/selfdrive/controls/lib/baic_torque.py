import numpy as np


# The X7 has about 0.34 s of measured steering response delay. Reduce only
# small-signal corrections and restore the standard gain before curve demand.
BAIC_STRAIGHT_KP_SCALE = 0.35
BAIC_STRAIGHT_LAT_ACCEL = 0.25
BAIC_FULL_KP_LAT_ACCEL = 0.75


def lateral_error_scale(is_baic: bool, desired_lat_accel: float, actual_lat_accel: float) -> float:
  if not is_baic:
    return 1.0

  lateral_demand = max(abs(desired_lat_accel), abs(actual_lat_accel))
  return float(np.interp(lateral_demand,
                         [BAIC_STRAIGHT_LAT_ACCEL, BAIC_FULL_KP_LAT_ACCEL],
                         [BAIC_STRAIGHT_KP_SCALE, 1.0]))
