import time

from openpilot.system.sensord import sensord_qmi
from openpilot.system.sensord.sensord_qmi import SENSOR_STREAM_TIMEOUT_NS, stalled_sensor_streams


def test_sensor_stream_watchdog_all_fresh():
  now_ns = 10 * SENSOR_STREAM_TIMEOUT_NS
  last_fresh_event_ns = {
    "accelerometer": now_ns - SENSOR_STREAM_TIMEOUT_NS,
    "gyroscope": now_ns,
  }

  assert stalled_sensor_streams(last_fresh_event_ns, now_ns) == []


def test_sensor_stream_watchdog_detects_each_stream_independently():
  now_ns = 10 * SENSOR_STREAM_TIMEOUT_NS
  last_fresh_event_ns = {
    "accelerometer": now_ns,
    "gyroscope": now_ns - SENSOR_STREAM_TIMEOUT_NS - 1,
  }

  assert stalled_sensor_streams(last_fresh_event_ns, now_ns) == ["gyroscope"]


def test_sensor_stream_watchdog_detects_full_qmi_stall():
  now_ns = 10 * SENSOR_STREAM_TIMEOUT_NS
  last_fresh_event_ns = {
    "accelerometer": now_ns - SENSOR_STREAM_TIMEOUT_NS - 1,
    "gyroscope": now_ns - SENSOR_STREAM_TIMEOUT_NS - 1,
  }

  assert stalled_sensor_streams(last_fresh_event_ns, now_ns) == ["accelerometer", "gyroscope"]


def test_silent_qmi_session_requests_restart(monkeypatch):
  class SilentQMIClient:
    def connect(self):
      pass

    def lookup_suid(self, data_type):
      return (1, 2) if data_type == "accel" else (3, 4)

    def start_streaming(self, _suid_low, _suid_high, _sample_rate):
      return True

    def recv_sensor_event(self, timeout_ms):
      time.sleep(timeout_ms / 1000)
      return None

    def close(self):
      pass

  monkeypatch.setattr(sensord_qmi, "SNSQMIClient", SilentQMIClient)
  monkeypatch.setattr(sensord_qmi, "running", True)

  started = time.monotonic()
  assert sensord_qmi.run_sensord(test_mode=True) == sensord_qmi.SENSOR_STREAM_STALLED
  assert time.monotonic() - started < 2.0
