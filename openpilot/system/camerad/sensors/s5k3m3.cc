#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <unistd.h>

#include "system/camerad/sensors/sensor.h"

namespace {

// Samsung analog gain code is linear: register code = real gain * 32. Xiaomi's
// factory library clamps analog gain at 16x (0x0200). Keep the upper analog
// range and the bounded digital tail dense enough that AE can hold integration
// close to the 50 ms ceiling instead of jumping from 16x straight to 20x and
// shortening exposure by roughly 20%. Longer integration collects more real
// photons and improves SNR; digital gain is used only for the remaining lift.
const float s5k3m3_analog_gains[] = {
  1.00f, 1.25f, 1.50f, 2.00f, 2.50f, 3.00f, 4.00f,
  5.00f, 6.00f, 8.00f, 10.00f, 12.00f, 14.00f, 16.00f,
  18.00f, 20.00f, 24.00f, 28.00f, 32.00f,
};

const uint16_t s5k3m3_analog_gain_codes[] = {
  0x0020, 0x0028, 0x0030, 0x0040, 0x0050, 0x0060, 0x0080,
  0x00A0, 0x00C0, 0x0100, 0x0140, 0x0180, 0x01C0, 0x0200,
};

constexpr int s5k3m3_max_analog_gain_idx = std::size(s5k3m3_analog_gain_codes) - 1;

// S5K3M3's factory analog ceiling is 16x. The extra 18x..32x table entries are
// a bounded digital tail, not additional sensor sensitivity. Penalize only
// that tail so AE collects real photons first, matching the C3X exposure
// policy as closely as this non-HDR phone sensor allows. The tail remains
// available when the 20 Hz integration ceiling is genuinely insufficient.
constexpr float s5k3m3_digital_gain_cost = 64.0f;

}  // namespace

S5K3M3::S5K3M3() {
  image_sensor = cereal::FrameData::ImageSensor::UNKNOWN;
  // The vendor mode declares BAYER_GRBG (enum value 2 in the CamX XML).
  bayer_pattern = CAM_ISP_PATTERN_BAYER_GRGRGR;
  // The selected 4208x3120 -> 2104x1560 mode is 2x2 binned. Use the
  // effective pitch so camerad's focal-pixel and AE ROI geometry are correct.
  pixel_size_mm = 0.0020f;
  data_word = true;         // Samsung tables use 16-bit register data

  frame_width = 2104;
  frame_height = 1560;
  frame_stride = frame_width * 10 / 8;
  out_scale = 1;
  extra_height = 0;
  frame_offset = 0;

  init_reg_array.assign(std::begin(s5k3m3_global_init), std::end(s5k3m3_global_init));
  init_reg_array.insert(init_reg_array.end(),
                        std::begin(s5k3m3_mode1_2104x1560_30fps),
                        std::end(s5k3m3_mode1_2104x1560_30fps));
  const bool dual_camera = getenv("ENABLE_TELE") != nullptr ||
                           access("/data/enable_mi8_tele_camera", F_OK) == 0;
  if (dual_camera) {
    // Nominal 20 Hz timing. Mi 8's two independent sensor PLLs drift relative
    // to each other, so camerad trims this FLL slowly around the nominal value.
    init_reg_array.push_back({0x0340, 0x0D64});
    init_reg_array.push_back({0x0342, 0x1B5E});
  }
  start_reg_array.assign(std::begin(start_reg_array_s5k3m3), std::end(start_reg_array_s5k3m3));

  probe_reg_addr = 0x0002;
  probe_expected_data = 0xB001;
  bits_per_pixel = 10;
  mipi_format = CAM_FORMAT_MIPI_RAW_10;
  frame_data_type = 0x2B;
  mclk_frequency = 24000000;
  readout_time_ns = 25000000;

  exposure_time_min = 2;
  // Leave headroom for the phase lock's minimum temporary FLL (3408).
  exposure_time_max = dual_camera ? 3388 : 2262;

  ev_scale = 1.0f;
  dc_gain_factor = 1.0f;
  dc_gain_min_weight = 1;
  dc_gain_max_weight = 1;
  dc_gain_on_grey = 0.9f;
  dc_gain_off_grey = 1.0f;
  analog_gain_min_idx = 0;
  analog_gain_rec_idx = 0;
  analog_gain_max_idx = std::size(s5k3m3_analog_gains) - 1;
  analog_gain_cost_delta = -1;
  analog_gain_cost_low = 0.4f;
  analog_gain_cost_high = 6.4f;
  for (int i = 0; i <= analog_gain_max_idx; ++i) {
    sensor_analog_gains[i] = s5k3m3_analog_gains[i];
  }
  min_ev = exposure_time_min * sensor_analog_gains[analog_gain_min_idx];
  max_ev = exposure_time_max * sensor_analog_gains[analog_gain_max_idx];
  target_grey_factor = 0.002f;

  // The IFE payload below is generated from Xiaomi's Semco S5K3M3 Chromatix,
  // not inherited from the IMX363.  The checked conversion resolves the QTI
  // symbol tree and validates the same CCM/Gamma15/Linearization33/LSC34/ABF34
  // conversion byte-for-byte against the existing Semco IMX363 tables first.
  black_level = 64;
#include "s5k3m3_ife.inc"
}

std::vector<i2c_random_wr_payload> S5K3M3::getExposureRegisters(int exposure_time, int new_exp_g, bool dc_gain_enabled) const {
  const int gain_idx = std::clamp(new_exp_g, analog_gain_min_idx, analog_gain_max_idx);
  const int analog_gain_idx = std::min(gain_idx, s5k3m3_max_analog_gain_idx);
  const uint16_t analog_gain = s5k3m3_analog_gain_codes[analog_gain_idx];
  const float digital_gain = sensor_analog_gains[gain_idx] / sensor_analog_gains[analog_gain_idx];
  const uint16_t digital_gain_code = static_cast<uint16_t>(
    std::clamp(std::lround(digital_gain * 256.0f), 0x0100L, 0x0200L));
  const uint16_t coarse = static_cast<uint16_t>(std::clamp(exposure_time, exposure_time_min, exposure_time_max));

  // Use the sensor's factory indirect group-hold sequence and apply the same
  // digital gain to all Bayer channels. Colour remains the calibrated IFE WB's
  // responsibility; this stage only adds luma after analog gain saturates.
  return {
    {0x6028, 0x4000}, {0x602A, 0x0104}, {0x6F12, 0x0100},
    {0x0202, coarse},
    {0x0204, analog_gain},
    {0x020E, digital_gain_code}, {0x0210, digital_gain_code},
    {0x0212, digital_gain_code}, {0x0214, digital_gain_code},
    {0x6028, 0x4000}, {0x602A, 0x0104}, {0x6F12, 0x0000},
  };
}

int S5K3M3::getSlaveAddress(int port) const {
  assert(port == 1);
  return 0x20;
}

float S5K3M3::getExposureScore(float desired_ev, int exp_t, int exp_g_idx, float exp_gain, int gain_idx) const {
  float score = std::abs(desired_ev - (exp_t * exp_gain));
  const float gain_cost = exp_g_idx > analog_gain_rec_idx ? analog_gain_cost_high : analog_gain_cost_low;
  score += std::abs(exp_g_idx - analog_gain_rec_idx) * gain_cost;
  const float digital_gain = std::max(exp_gain / s5k3m3_analog_gains[s5k3m3_max_analog_gain_idx], 1.0f);
  score += (digital_gain - 1.0f) * s5k3m3_digital_gain_cost;
  score += ((1 - analog_gain_cost_delta) +
            analog_gain_cost_delta * (exp_g_idx - analog_gain_min_idx) /
              static_cast<float>(analog_gain_max_idx - analog_gain_min_idx)) *
           std::abs(exp_g_idx - gain_idx) * 3.0f;
  return score;
}
