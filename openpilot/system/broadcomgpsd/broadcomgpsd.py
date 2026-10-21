#!/usr/bin/env python3
"""Publish Mi8 BCM47755 NMEA fixes as openpilot gpsLocationExternal."""

import calendar
import collections
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
import traceback

from openpilot.cereal import log, messaging
from openpilot.common.swaglog import cloudlog


ANDROID_ROOT = Path("/data/android_root")
CONFIG_ROOT = Path("/data/openpilot/system/broadcomgpsd")
GPS_PIPE = ANDROID_ROOT / "data/vendor/gps/gpspipe"
CHROOT_CONFIG_ROOT = "/data/vendor/gps/openpilot"
RUNTIME_LOG_ROOT = Path("/data/local/tmp/x7-broadcomgpsd-runtime-comma")
GPS_WEEK_ROLLOVER_MS = 1024 * 7 * 24 * 60 * 60 * 1000
GPS_TIME_SANITY_WINDOW_MS = 180 * 24 * 60 * 60 * 1000
VENDOR_FIX_ACQUIRE_COUNT = 3
VENDOR_FIX_ACQUIRE_WINDOW_S = 5.0
VENDOR_FIX_RELEASE_S = 2.0
STATUS_LOG_INTERVAL_S = 60.0


def nmea_degrees(value: str, hemisphere: str) -> float:
  raw = float(value)
  degrees = int(raw // 100)
  result = degrees + (raw - degrees * 100) / 60.0
  return -result if hemisphere in ("S", "W") else result


def valid_checksum(sentence: str) -> bool:
  if not sentence.startswith("$") or "*" not in sentence:
    return False
  body, expected = sentence[1:].split("*", 1)
  checksum = 0
  for char in body:
    checksum ^= ord(char)
  try:
    return checksum == int(expected[:2], 16)
  except ValueError:
    return False


def parse_nmea(sentence: str) -> dict | None:
  sentence = sentence.strip()
  if not valid_checksum(sentence):
    return None
  fields = sentence[1:sentence.index("*")].split(",")
  kind = fields[0][-3:]
  try:
    if kind == "GGA" and len(fields) >= 10:
      return {
        "kind": "GGA",
        "latitude": nmea_degrees(fields[2], fields[3]),
        "longitude": nmea_degrees(fields[4], fields[5]),
        "quality": int(fields[6] or 0),
        "satellites": int(fields[7] or 0),
        "hdop": float(fields[8] or 99.0),
        "altitude": float(fields[9] or 0.0),
      }
    if kind == "RMC" and len(fields) >= 10:
      utc = fields[1]
      date = fields[9]
      has_position = all(fields[index] for index in (3, 4, 5, 6))
      has_timestamp = len(utc) >= 6 and len(date) == 6
      timestamp_ms = 0
      if has_timestamp:
        day, month, year = int(date[:2]), int(date[2:4]), int(date[4:6])
        year += 2000 if year < 80 else 1900
        hour, minute = int(utc[:2]), int(utc[2:4])
        seconds = float(utc[4:])
        sec_int = int(seconds)
        timestamp_ms = calendar.timegm((year, month, day, hour, minute, sec_int, 0, 0, 0)) * 1000
        timestamp_ms += int(round((seconds - sec_int) * 1000))
      return {
        "kind": "RMC",
        # BCM47755 emits status=V RMC sentences with blank position/date while
        # searching. Preserve those as invalid heartbeat messages instead of
        # silently dropping the entire gpsLocationExternal service.
        "has_fix": fields[2] == "A" and has_position and has_timestamp,
        "latitude": nmea_degrees(fields[3], fields[4]) if has_position else 0.0,
        "longitude": nmea_degrees(fields[5], fields[6]) if has_position else 0.0,
        "speed": float(fields[7] or 0.0) * 0.514444,
        "bearing": float(fields[8] or 0.0),
        "timestamp_ms": timestamp_ms,
      }
  except (IndexError, TypeError, ValueError):
    return None
  return None


def normalize_gps_timestamp(timestamp_ms: int, system_time_ms: int | None = None) -> int:
  """Correct an integer number of GPS week rollovers using a trusted system clock."""
  if system_time_ms is None:
    system_time_ms = time.time_ns() // 1_000_000
  if timestamp_ms <= 0 or system_time_ms <= 0:
    return timestamp_ms

  rollover_count = round((system_time_ms - timestamp_ms) / GPS_WEEK_ROLLOVER_MS)
  if rollover_count <= 0:
    return timestamp_ms

  corrected_timestamp_ms = timestamp_ms + rollover_count * GPS_WEEK_ROLLOVER_MS
  if abs(corrected_timestamp_ms - system_time_ms) <= GPS_TIME_SANITY_WINDOW_MS:
    return corrected_timestamp_ms
  return timestamp_ms


def publish_fix(pm: messaging.PubMaster, rmc: dict, gga: dict | None) -> None:
  bearing_rad = math.radians(rmc["bearing"])
  hdop = gga["hdop"] if gga is not None else 99.0
  message = messaging.new_message("gpsLocationExternal", valid=bool(rmc["has_fix"]))
  gps = message.gpsLocationExternal
  gps.source = log.GpsLocationData.SensorSource.external
  gps.flags = 1 if rmc["has_fix"] else 0
  gps.hasFix = bool(rmc["has_fix"])
  gps.latitude = rmc["latitude"]
  gps.longitude = rmc["longitude"]
  gps.altitude = gga["altitude"] if gga is not None else 0.0
  gps.speed = rmc["speed"]
  gps.bearingDeg = rmc["bearing"]
  gps.horizontalAccuracy = max(1.0, hdop * 5.0)
  gps.verticalAccuracy = max(2.0, hdop * 8.0)
  gps.speedAccuracy = 0.5
  gps.bearingAccuracyDeg = 5.0
  gps.unixTimestampMillis = normalize_gps_timestamp(rmc["timestamp_ms"])
  gps.satelliteCount = min(127, gga["satellites"] if gga is not None else 0)
  gps.vNED = [
    rmc["speed"] * math.cos(bearing_rad),
    rmc["speed"] * math.sin(bearing_rad),
    0.0,
  ]
  pm.send("gpsLocationExternal", message)


def publish_phone_fix(pm: messaging.PubMaster, carrot) -> None:
  """Use NaviPilot/VietMap phone GPS while the BCM receiver has no fix."""
  bearing_deg = float(carrot.xPosAngle) % 360.0
  speed = max(0.0, float(carrot.xPosSpeed) / 3.6)
  bearing_rad = math.radians(bearing_deg)
  message = messaging.new_message("gpsLocationExternal", valid=True)
  gps = message.gpsLocationExternal
  # Do not label a NaviPilot fallback as BCM hardware. Both sources share the
  # gpsLocationExternal cereal service, so the source field is the only way a
  # route audit can distinguish them without relying on satelliteCount == 0.
  gps.source = log.GpsLocationData.SensorSource.android
  gps.flags = 1
  gps.hasFix = True
  gps.latitude = float(carrot.xPosLat)
  gps.longitude = float(carrot.xPosLon)
  gps.altitude = 0.0
  gps.speed = speed
  gps.bearingDeg = bearing_deg
  gps.horizontalAccuracy = 10.0
  gps.verticalAccuracy = 25.0
  gps.speedAccuracy = 1.0
  gps.bearingAccuracyDeg = 10.0
  gps.unixTimestampMillis = time.time_ns() // 1_000_000
  gps.satelliteCount = 0
  gps.vNED = [speed * math.cos(bearing_rad), speed * math.sin(bearing_rad), 0.0]
  pm.send("gpsLocationExternal", message)


def terminate(process: subprocess.Popen | None) -> None:
  if process is None or process.poll() is not None:
    return
  try:
    os.killpg(process.pid, signal.SIGTERM)
    process.wait(timeout=5)
  except PermissionError:
    subprocess.run(["sudo", "-n", "kill", "-TERM", f"-{process.pid}"], check=False)
    try:
      process.wait(timeout=5)
    except subprocess.TimeoutExpired:
      subprocess.run(["sudo", "-n", "kill", "-KILL", f"-{process.pid}"], check=False)
  except (ProcessLookupError, subprocess.TimeoutExpired):
    try:
      os.killpg(process.pid, signal.SIGKILL)
    except PermissionError:
      subprocess.run(["sudo", "-n", "kill", "-KILL", f"-{process.pid}"], check=False)
    except ProcessLookupError:
      pass


def vendor_process_running(name: str) -> bool:
  """Return whether a vendor GNSS child already exists inside the chroot."""
  needle = f"/vendor/bin/{name}".encode()
  for proc in Path("/proc").glob("[0-9]*/cmdline"):
    try:
      if needle in proc.read_bytes():
        return True
    except (FileNotFoundError, PermissionError, ProcessLookupError):
      pass
  return False


def run_as_root() -> None:
  lhd = None
  glgps = None
  owns_lhd = False
  owns_glgps = False
  lhd_output = None
  glgps_output = None
  try:
    RUNTIME_LOG_ROOT.mkdir(parents=True, exist_ok=True)
    try:
      GPS_PIPE.unlink()
    except FileNotFoundError:
      pass
    lhd_output = open(RUNTIME_LOG_ROOT / "lhd.log", "wb", buffering=0)
    glgps_output = open(RUNTIME_LOG_ROOT / "glgps.log", "wb", buffering=0)
    root_prefix = [] if os.geteuid() == 0 else ["sudo", "-n"]
    if not vendor_process_running("lhd"):
      lhd = subprocess.Popen(root_prefix + [
        "chroot", str(ANDROID_ROOT), "/vendor/bin/lhd",
        f"{CHROOT_CONFIG_ROOT}/lhd.conf",
      ], stdout=lhd_output, stderr=subprocess.STDOUT, start_new_session=True)
      owns_lhd = True
      time.sleep(3.0)
      if lhd.poll() is not None:
        raise RuntimeError(f"lhd exited with {lhd.returncode}")
    else:
      cloudlog.info("Reusing existing Mi8 lhd process")

    if not vendor_process_running("glgps"):
      glgps = subprocess.Popen(root_prefix + [
        "chroot", str(ANDROID_ROOT), "/vendor/bin/glgps",
        f"{CHROOT_CONFIG_ROOT}/gpsconfig.xml", "Periodic",
      ], stdout=glgps_output, stderr=subprocess.STDOUT, start_new_session=True)
      owns_glgps = True
    else:
      cloudlog.info("Reusing existing Mi8 glgps process")

    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline and not GPS_PIPE.exists():
      if not vendor_process_running("glgps"):
        returncode = glgps.returncode if glgps is not None else "unknown"
        raise RuntimeError(f"glgps exited with {returncode}")
      time.sleep(0.25)
    if not GPS_PIPE.exists():
      raise RuntimeError("gpspipe was not created")

    pm = messaging.PubMaster(["gpsLocationExternal"])
    sm = messaging.SubMaster(["carrotMan"])
    fd = os.open(GPS_PIPE, os.O_RDONLY | os.O_NONBLOCK)
    poller = select.poll()
    poller.register(fd, select.POLLIN | select.POLLHUP | select.POLLERR)
    buffer = b""
    last_gga = None
    last_gga_time = 0.0
    last_vendor_fix_time = 0.0
    vendor_fix_times = collections.deque()
    vendor_active = False
    pending_vendor_rmc = None
    pending_vendor_gga = None
    last_phone_publish_time = 0.0
    last_vendor_process_check = 0.0
    last_status_log_time = time.monotonic()
    nmea_lines = 0
    gga_count = 0
    rmc_count = 0
    valid_rmc_count = 0
    max_satellites = 0
    try:
      while True:
        now = time.monotonic()
        if now - last_vendor_process_check >= 1.0:
          if not vendor_process_running("lhd"):
            raise RuntimeError("Broadcom GNSS lhd exited")
          if not vendor_process_running("glgps"):
            raise RuntimeError("Broadcom GNSS glgps exited")
          last_vendor_process_check = now
        for _, event in poller.poll(20):
          if event & select.POLLIN:
            chunk = os.read(fd, 8192)
            if not chunk:
              continue
            buffer += chunk
            lines = buffer.split(b"\n")
            buffer = lines.pop()
            for raw in lines:
              nmea_lines += 1
              parsed = parse_nmea(raw.decode("ascii", errors="ignore"))
              if parsed is None:
                continue
              now = time.monotonic()
              if parsed["kind"] == "GGA":
                gga_count += 1
                max_satellites = max(max_satellites, parsed["satellites"])
                last_gga = parsed
                last_gga_time = now
              elif parsed["kind"] == "RMC":
                rmc_count += 1
                recent_gga = last_gga if now - last_gga_time < 2.5 else None
                if recent_gga is not None:
                  parsed["has_fix"] = parsed["has_fix"] and recent_gga["quality"] > 0
                if parsed["has_fix"]:
                  valid_rmc_count += 1
                  last_vendor_fix_time = now
                  vendor_fix_times.append(now)
                pending_vendor_rmc = parsed
                pending_vendor_gga = recent_gga
          elif event & (select.POLLHUP | select.POLLERR):
            time.sleep(0.1)

        # The Android navigation app already supplies a filtered position at
        # 10 Hz. Keep Google/OSM/map-speed logic stable indoors and during a
        # BCM cold start, but hand priority back to the hardware receiver as
        # soon as it has a recent valid fix.
        sm.update(0)
        now = time.monotonic()
        phone = sm["carrotMan"]
        phone_valid = (sm.alive["carrotMan"] and sm.valid["carrotMan"] and
                       phone.activeCarrot > 0 and bool(phone.remote) and
                       abs(float(phone.xPosLat)) <= 90.0 and abs(float(phone.xPosLon)) <= 180.0 and
                       abs(float(phone.xPosLat)) > 1e-6 and abs(float(phone.xPosLon)) > 1e-6)

        while vendor_fix_times and now - vendor_fix_times[0] > VENDOR_FIX_ACQUIRE_WINDOW_S:
          vendor_fix_times.popleft()

        # Some BCM firmware builds interleave a void constellation-specific
        # RMC sentence with the valid combined RMC. Requiring strictly
        # consecutive valid sentences can therefore hide a real hardware fix
        # forever. Acquire on a bounded window of valid fixes instead.
        if not vendor_active and len(vendor_fix_times) >= VENDOR_FIX_ACQUIRE_COUNT:
          vendor_active = True
        elif vendor_active and now - last_vendor_fix_time > VENDOR_FIX_RELEASE_S:
          vendor_active = False

        if now - last_status_log_time >= STATUS_LOG_INTERVAL_S:
          cloudlog.info(
            "BCM47755 NMEA status lines=%d gga=%d rmc=%d valid_rmc=%d max_sats=%d vendor_active=%s phone_fallback=%s",
            nmea_lines, gga_count, rmc_count, valid_rmc_count, max_satellites,
            vendor_active, phone_valid and not vendor_active,
          )
          last_status_log_time = now

        if vendor_active and pending_vendor_rmc is not None and pending_vendor_rmc["has_fix"]:
          publish_fix(pm, pending_vendor_rmc, pending_vendor_gga)
          pending_vendor_rmc = None
          pending_vendor_gga = None
        elif not vendor_active and phone_valid and now - last_phone_publish_time >= 0.09:
          publish_phone_fix(pm, phone)
          last_phone_publish_time = now
        elif not vendor_active and not phone_valid and pending_vendor_rmc is not None:
          publish_fix(pm, pending_vendor_rmc, pending_vendor_gga)
          pending_vendor_rmc = None
          pending_vendor_gga = None
    finally:
      os.close(fd)
  except BaseException:
    # Append so a later successful manager/runtime restart does not erase the
    # reason for an earlier vendor-child exit.
    with open(RUNTIME_LOG_ROOT / "bridge-error.log", "a") as error_output:
      print(f"\n===== {time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())} =====", file=error_output)
      traceback.print_exc(file=error_output)
    raise
  finally:
    if owns_glgps:
      terminate(glgps)
    if owns_lhd:
      terminate(lhd)
    if glgps_output is not None:
      glgps_output.close()
    if lhd_output is not None:
      lhd_output.close()


def self_test() -> None:
  gga = parse_nmea("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47")
  rmc = parse_nmea("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A")
  assert gga is not None and rmc is not None
  assert abs(gga["latitude"] - 48.1173) < 1e-6
  assert abs(gga["longitude"] - 11.5166667) < 1e-6
  assert gga["quality"] == 1 and gga["satellites"] == 8
  assert abs(rmc["speed"] - 11.5235456) < 1e-6
  assert rmc["timestamp_ms"] == 764426119000
  no_fix_rmc = parse_nmea("$GPRMC,002420.00,V,,,,,,,,,,N*79")
  assert no_fix_rmc is not None
  assert not no_fix_rmc["has_fix"]
  assert no_fix_rmc["latitude"] == 0.0 and no_fix_rmc["longitude"] == 0.0
  assert no_fix_rmc["timestamp_ms"] == 0
  rollover_timestamp_ms = 1172377497000
  current_timestamp_ms = rollover_timestamp_ms + GPS_WEEK_ROLLOVER_MS
  assert normalize_gps_timestamp(rollover_timestamp_ms, current_timestamp_ms) == current_timestamp_ms
  assert normalize_gps_timestamp(current_timestamp_ms, current_timestamp_ms) == current_timestamp_ms
  assert normalize_gps_timestamp(rmc["timestamp_ms"], current_timestamp_ms) == rmc["timestamp_ms"]
  print("BROADCOM_GPSD_SELF_TEST_OK")


def main() -> None:
  if "--self-test" in sys.argv:
    self_test()
    return
  # The Mi 8 vendor glgps binary ends its Periodic job after roughly 1,000
  # valid fixes even when a larger ValidFixes value is supplied in XML. Keep
  # the cereal publisher process stable and restart the isolated vendor stack
  # locally, instead of making manager restart the whole bridge and leaving a
  # longer gpsLocationExternal/map gap. A failure after at least one minute is
  # treated as a fresh lifecycle, while rapid failures receive bounded
  # exponential backoff.
  consecutive_failures = 0
  while True:
    started = time.monotonic()
    cloudlog.info("Starting Mi8 BCM47755 GNSS vendor stack")
    try:
      run_as_root()
      consecutive_failures = 0
    except Exception:
      runtime = time.monotonic() - started
      if runtime >= 60.0:
        consecutive_failures = 0
      consecutive_failures += 1
      delay = min(30.0, float(2 ** min(consecutive_failures - 1, 5)))
      cloudlog.exception(
        "Mi8 BCM47755 GNSS vendor stack exited after %.1fs; restarting in %.1fs",
        runtime, delay,
      )
      time.sleep(delay)


if __name__ == "__main__":
  main()
