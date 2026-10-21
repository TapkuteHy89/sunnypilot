from openpilot.selfdrive.carrot.carrot_serv import CarrotServ


def make_serv():
  serv = CarrotServ.__new__(CarrotServ)
  serv.nRoadLimitSpeed = 0
  serv.road_limit_valid = False
  serv.road_limit_valid_until = 0.0
  serv.road_limit_source_timestamp_ms = 0
  return serv


def test_unrelated_route_packet_does_not_clear_vietmap_limit():
  serv = make_serv()
  serv._update_vietmap_road_limit({
    "roadLimitValid": True,
    "roadLimitSource": "vietmap_live",
    "roadLimitTimestampMs": 1_000_000,
    "nRoadLimitSpeed": 80,
  }, now_monotonic=100.0, now_wall_ms=1_000_000)

  serv._update_vietmap_road_limit({
    "navActive": True,
    "navSource": "google_nav",
    "routeGeometryValid": True,
  }, now_monotonic=101.0, now_wall_ms=1_001_000)
  serv._update_vietmap_road_limit({
    "nRoadLimitSpeed": 80,
    "timestamp_ms": 1_001_000,
  }, now_monotonic=101.0, now_wall_ms=1_001_000)

  assert serv.road_limit_valid
  assert serv.nRoadLimitSpeed == 80
  assert serv.road_limit_valid_until == 130.0


def test_unrelated_protocol_v2_empty_channel_does_not_clear_newer_vietmap_limit():
  serv = make_serv()
  serv._update_vietmap_road_limit({
    "roadLimitValid": True,
    "roadLimitSource": "vietmap_live",
    "roadLimitTimestampMs": 1_500_000,
    "nRoadLimitSpeed": 80,
  }, now_monotonic=150.0, now_wall_ms=1_500_000)

  serv._update_vietmap_road_limit({
    "roadLimitValid": False,
    "roadLimitSource": "none",
    "roadLimitTimestampMs": 0,
    "navSource": "google_nav",
  }, now_monotonic=151.0, now_wall_ms=1_501_000)

  assert serv.road_limit_valid
  assert serv.nRoadLimitSpeed == 80


def test_foreign_protocol_clear_cannot_revoke_vietmap():
  serv = make_serv()
  serv._update_vietmap_road_limit({
    "roadLimitValid": True,
    "roadLimitSource": "vietmap_live",
    "roadLimitTimestampMs": 2_000_000,
    "nRoadLimitSpeed": 50,
  }, now_monotonic=200.0, now_wall_ms=2_000_000)

  serv._update_vietmap_road_limit({
    "roadLimitValid": False,
    "roadLimitSource": "none",
    "roadLimitTimestampMs": 2_001_000,
  }, now_monotonic=201.0, now_wall_ms=2_001_000)

  assert serv.road_limit_valid
  assert serv.road_limit_valid_until == 230.0


def test_explicit_vietmap_protocol_clear_is_honored():
  serv = make_serv()
  serv._update_vietmap_road_limit({
    "roadLimitValid": True,
    "roadLimitSource": "vietmap_live",
    "roadLimitTimestampMs": 2_000_000,
    "nRoadLimitSpeed": 50,
  }, now_monotonic=200.0, now_wall_ms=2_000_000)

  serv._update_vietmap_road_limit({
    "roadLimitValid": False,
    "roadLimitSource": "vietmap_live",
    "roadLimitTimestampMs": 2_001_000,
  }, now_monotonic=201.0, now_wall_ms=2_001_000)

  assert not serv.road_limit_valid
  assert serv.road_limit_valid_until == 0.0


def test_stale_or_wrong_source_cannot_replace_valid_vietmap_limit():
  serv = make_serv()
  serv._update_vietmap_road_limit({
    "roadLimitValid": True,
    "roadLimitSource": "vietmap_live",
    "roadLimitTimestampMs": 3_000_000,
    "nRoadLimitSpeed": 60,
  }, now_monotonic=300.0, now_wall_ms=3_000_000)

  serv._update_vietmap_road_limit({
    "roadLimitValid": True,
    "roadLimitSource": "google_nav",
    "roadLimitTimestampMs": 3_001_000,
    "nRoadLimitSpeed": 100,
  }, now_monotonic=301.0, now_wall_ms=3_001_000)
  serv._update_vietmap_road_limit({
    "roadLimitValid": True,
    "roadLimitSource": "vietmap_live",
    "roadLimitTimestampMs": 2_960_000,
    "nRoadLimitSpeed": 40,
  }, now_monotonic=301.0, now_wall_ms=3_001_000)

  assert serv.road_limit_valid
  assert serv.nRoadLimitSpeed == 60
  assert serv.road_limit_valid_until == 330.0


def test_vietmap_limit_expires_without_a_refresh():
  serv = make_serv()
  serv._update_vietmap_road_limit({
    "roadLimitValid": True,
    "roadLimitSource": "vietmap_live",
    "roadLimitTimestampMs": 4_000_000,
    "nRoadLimitSpeed": 40,
  }, now_monotonic=400.0, now_wall_ms=4_000_000)

  serv._expire_vietmap_road_limit(now_monotonic=429.9)
  assert serv.road_limit_valid
  serv._expire_vietmap_road_limit(now_monotonic=430.0)
  assert not serv.road_limit_valid


def test_fresh_legacy_vietmap_packet_remains_supported():
  serv = make_serv()
  serv._update_vietmap_road_limit({
    "bridgeSource": "vietmap_live",
    "bridgeTimestampMs": 5_000_000,
    "nRoadLimitSpeed": 50,
  }, now_monotonic=500.0, now_wall_ms=5_001_000)

  assert serv.road_limit_valid
  assert serv.nRoadLimitSpeed == 50
  assert serv.road_limit_valid_until == 530.0


def test_phone_wall_clock_skew_does_not_shorten_receiver_lease():
  serv = make_serv()
  serv._update_vietmap_road_limit({
    "roadLimitValid": True,
    "roadLimitSource": "vietmap_live",
    "roadLimitTimestampMs": 9_000_000,
    "nRoadLimitSpeed": 80,
  }, now_monotonic=600.0, now_wall_ms=9_029_200)

  assert serv.road_limit_valid
  assert serv.nRoadLimitSpeed == 80
  assert serv.road_limit_valid_until == 630.0
