#include <cassert>
#include <cmath>
#include <iostream>

#include "sunnypilot/selfdrive/locationd/locationd.h"

static void send_fix(Localizer &localizer, double time, double bearing, bool valid = true) {
  MessageBuilder message;
  auto gps = message.initEvent().initGpsLocationExternal();
  gps.setHasFix(valid);
  gps.setLatitude(21.0);
  gps.setLongitude(105.0);
  gps.setAltitude(10.0);
  gps.setHorizontalAccuracy(1.0);
  gps.setVerticalAccuracy(1.0);
  gps.setSpeedAccuracy(0.1);
  gps.setBearingAccuracyDeg(1.0);
  gps.setBearingDeg(bearing);
  gps.setSatelliteCount(12);
  auto velocity = gps.initVNED(3);
  velocity.set(0, 10.0);
  velocity.set(1, 0.0);
  velocity.set(2, 0.0);
  localizer.handle_gps(time, gps.asReader(), 0.0);
}

static Eigen::VectorXd orientation(Localizer &localizer) {
  return localizer.get_state().segment<STATE_ECEF_ORIENTATION_LEN>(STATE_ECEF_ORIENTATION_START);
}

static void test_debounce(LocalizerGnssSource source, int samples) {
  Localizer localizer(source);
  double time = 1.0;
  send_fix(localizer, time, 0.0);
  const auto initial = orientation(localizer);
  for (int i = 1; i < samples; ++i) {
    send_fix(localizer, time += 0.1, 180.0);
    assert(std::abs(initial.dot(orientation(localizer))) > 0.99);
  }
  send_fix(localizer, time += 0.1, 180.0);
  assert(std::abs(initial.dot(orientation(localizer))) < 0.1);
}

static void test_interrupted_bad_course() {
  Localizer localizer(LocalizerGnssSource::BROADCOM);
  double time = 1.0;
  send_fix(localizer, time, 0.0);
  const auto initial = orientation(localizer);
  for (int i = 0; i < 5; ++i) send_fix(localizer, time += 0.1, 180.0);
  send_fix(localizer, time += 0.1, 180.0, false);
  for (int i = 0; i < 5; ++i) send_fix(localizer, time += 0.1, 180.0);
  assert(std::abs(initial.dot(orientation(localizer))) > 0.99);
  time += 2.0;
  for (int i = 0; i < 9; ++i) send_fix(localizer, time += 0.1, 180.0);
  assert(std::abs(initial.dot(orientation(localizer))) > 0.99);
  send_fix(localizer, time += 0.1, 180.0);
  assert(std::abs(initial.dot(orientation(localizer))) < 0.1);
}

int main() {
  test_debounce(LocalizerGnssSource::BROADCOM, 10);
  test_debounce(LocalizerGnssSource::UBLOX, 4);
  test_interrupted_bad_course();
  std::cout << "GPS reset regression checks passed\n";
}
