"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.constants import CV

BAIC_TARGET_LAT_ACCEL = 1.0  # m/s^2, shared BAIC X7 Vision and OSM curve-speed target
MIN_V = 20 * CV.KPH_TO_MS  # Do not operate under 20 km/h
