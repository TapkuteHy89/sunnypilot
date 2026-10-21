from openpilot.selfdrive.carrot.route_geometry import find_osm_turn


def test_finds_concentrated_left_intersection_turn():
  points = [(0., 0.), (10., 0.), (20., 0.), (20., -10.), (20., -20.), (20., -30.)]
  distances = [0., 10., 20., 30., 40., 50.]
  assert find_osm_turn(points, distances) == (12, 20)


def test_finds_concentrated_right_intersection_turn():
  points = [(0., 0.), (10., 0.), (20., 0.), (20., 10.), (20., 20.), (20., 30.)]
  distances = [0., 10., 20., 30., 40., 50.]
  assert find_osm_turn(points, distances) == (13, 20)


def test_ignores_shallow_road_curve_and_uturn():
  shallow = [(0., 0.), (10., 0.), (20., 1.), (30., 4.), (40., 8.), (50., 13.)]
  uturn = [(0., 0.), (10., 0.), (20., 0.), (10., 0.), (0., 0.), (-10., 0.)]
  distances = [0., 10., 20., 30., 40., 50.]
  assert find_osm_turn(shallow, distances) is None
  assert find_osm_turn(uturn, distances) is None
