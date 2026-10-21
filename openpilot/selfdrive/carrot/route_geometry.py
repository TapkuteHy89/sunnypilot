import math


OSM_TURN_MIN_ANGLE_DEG = 50.0
OSM_TURN_MAX_ANGLE_DEG = 135.0
OSM_TURN_MIN_DISTANCE_M = 20.0
OSM_TURN_MAX_DISTANCE_M = 220.0


def resample_polyline(points: list[tuple[float, float]], interval_m: float) -> tuple[list[tuple[float, float]], list[float]]:
  """Resample a polyline at fixed distances without an external geometry dependency."""
  if not points or interval_m <= 0.:
    return [], []

  resampled_points = [points[0]]
  resampled_distances = [0.]
  distance_at_segment_start = 0.
  next_sample_distance = interval_m

  for start, end in zip(points, points[1:], strict=False):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    segment_length = math.hypot(dx, dy)
    if segment_length <= 1e-6:
      continue

    segment_end_distance = distance_at_segment_start + segment_length
    while next_sample_distance <= segment_end_distance + 1e-6:
      ratio = min(1., (next_sample_distance - distance_at_segment_start) / segment_length)
      resampled_points.append((start[0] + ratio * dx, start[1] + ratio * dy))
      resampled_distances.append(next_sample_distance)
      next_sample_distance += interval_m

    distance_at_segment_start = segment_end_distance

  return resampled_points, resampled_distances


def find_osm_turn(points, distances):
  """Find the next concentrated left/right turn in a local OSM route."""
  if len(points) < 5 or len(points) != len(distances):
    return None

  for i in range(2, len(points) - 2):
    distance = float(distances[i])
    if distance < OSM_TURN_MIN_DISTANCE_M:
      continue
    if distance > OSM_TURN_MAX_DISTANCE_M:
      break
    heading_in = math.atan2(points[i][1] - points[i - 2][1], points[i][0] - points[i - 2][0])
    heading_out = math.atan2(points[i + 2][1] - points[i][1], points[i + 2][0] - points[i][0])
    delta = math.degrees(math.atan2(math.sin(heading_out - heading_in), math.cos(heading_out - heading_in)))
    if OSM_TURN_MIN_ANGLE_DEG <= abs(delta) <= OSM_TURN_MAX_ANGLE_DEG:
      # gps_to_relative_xy uses (forward, right), so positive angular change
      # is a right turn rather than the left turn of a conventional x/y plane.
      return (13 if delta > 0 else 12), int(round(distance))
  return None
