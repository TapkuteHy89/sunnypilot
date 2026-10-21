#include "system/camerad/cameras/camera_common.h"
#include "system/camerad/cameras/spectra.h"

#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "common/params.h"
#include "common/swaglog.h"


ExitHandler do_exit;

// for debugging
const bool env_debug_frames = getenv("DEBUG_FRAMES") != nullptr;
const bool env_log_raw_frames = getenv("LOG_RAW_FRAMES") != nullptr;
const bool env_ctrl_exp_from_params = getenv("CTRL_EXP_FROM_PARAMS") != nullptr;

static int ae_update_interval() {
  const char *value = getenv("CAMERA_AE_UPDATE_INTERVAL");
  if (value == nullptr) {
    const bool dual_camera = getenv("ENABLE_TELE") != nullptr ||
                             access("/data/enable_mi8_tele_camera", F_OK) == 0;
    return dual_camera ? 20 : 1;
  }

  char *end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  return (end != value && *end == '\0' && parsed >= 1 && parsed <= 20)
    ? static_cast<int>(parsed) : 1;
}

static int tele_start_delay_us() {
  constexpr int default_delay_us = 29000;
  const char *value = getenv("MI8_TELE_START_DELAY_US");
  if (value == nullptr) return default_delay_us;

  char *end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  return (end != value && *end == '\0' && parsed >= 0 && parsed <= 60000)
    ? static_cast<int>(parsed) : default_delay_us;
}

// The sensor applies an exposure command three frames later. A 50 ms AE time
// constant over-corrects that delayed feedback and is visibly unstable when a
// moving scene changes the metering area. Keep this configurable for tuning,
// while using a response that settles smoothly over several camera frames.
static float ae_response_seconds() {
  constexpr float default_seconds = 0.20f;
  const char *value = getenv("IMX363_AE_RESPONSE_SECONDS");
  if (value == nullptr) return default_seconds;

  char *end = nullptr;
  const float parsed = std::strtof(value, &end);
  return (end != value && *end == '\0' && std::isfinite(parsed) && parsed >= 0.05f && parsed <= 2.0f)
    ? parsed : default_seconds;
}

static float ae_grey_deadband() {
  constexpr float default_fraction = 0.035f;
  const char *value = getenv("IMX363_AE_GREY_DEADBAND");
  if (value == nullptr) return default_fraction;

  char *end = nullptr;
  const float parsed = std::strtof(value, &end);
  return (end != value && *end == '\0' && std::isfinite(parsed) && parsed >= 0.005f && parsed <= 0.15f)
    ? parsed : default_fraction;
}

// Dual-camera exposure writes stay staggered because the dipper CCI/debug
// driver is unreliable when both sensors submit command packets together.
// Four frames gives each camera one private slot every 200 ms (camera 0 at
// phase 0, camera 1 at phase 2) while a bright-exit recovery is active.
constexpr int FAST_EXIT_INTERVAL_FRAMES = 4;
constexpr int FAST_EXIT_CONFIRM_FRAMES = 2;
constexpr int FAST_EXIT_MONITOR_SKIP = 8;

static float ae_fast_exit_ratio() {
  // The accepted tunnel route peaked at 1.15x (IMX363) and 1.17x (S5K3M3).
  // 1.12x catches both exits while remaining well outside the 3.5% steady AE
  // deadband. Two consecutive frames below make this resistant to one-frame
  // histogram noise.
  constexpr float default_ratio = 1.12f;
  const char *value = getenv("MI8_AE_FAST_EXIT_RATIO");
  if (value == nullptr) return default_ratio;

  char *end = nullptr;
  const float parsed = std::strtof(value, &end);
  return (end != value && *end == '\0' && std::isfinite(parsed) && parsed >= 1.08f && parsed <= 3.0f)
    ? parsed : default_ratio;
}

static float ae_fast_exit_delta() {
  constexpr float default_fraction = 0.035f;
  const char *value = getenv("MI8_AE_FAST_EXIT_DELTA");
  if (value == nullptr) return default_fraction;

  char *end = nullptr;
  const float parsed = std::strtof(value, &end);
  return (end != value && *end == '\0' && std::isfinite(parsed) && parsed >= 0.025f && parsed <= 0.25f)
    ? parsed : default_fraction;
}


class CameraState {
public:
  SpectraCamera camera;
  int exposure_time = 5;
  bool dc_gain_enabled = false;
  int dc_gain_weight = 0;
  int gain_idx = 0;
  float analog_gain_frac = 0;

  float cur_ev[3] = {};
  float best_ev_score = 0;
  int new_exp_g = 0;
  int new_exp_t = 0;

  Rect ae_xywh = {};
  float measured_grey_fraction = 0;
  float target_grey_fraction = 0.125;
  uint64_t last_exposure_update_frame = 0;
  int overexposed_frame_count = 0;

  float fl_pix = 0;
  std::unique_ptr<PubMaster> pm;

  CameraState(SpectraMaster *master, const CameraConfig &config) : camera(master, config) {};
  ~CameraState();
  void init(VisionIpcServer *v);
  void update_exposure_score(float desired_ev, int exp_t, int exp_g_idx, float exp_gain,
                             bool ignore_transition_cost = false);
  void set_camera_exposure(float grey_frac, int update_interval, bool fast_recovery = false);
  void set_exposure_rect();
  void sendState();

  float get_gain_factor() const {
    return (1 + dc_gain_weight * (camera.sensor->dc_gain_factor-1) / camera.sensor->dc_gain_max_weight);
  }
};

void CameraState::init(VisionIpcServer *v) {
  camera.camera_open(v);

  if (!camera.enabled) return;

  fl_pix = camera.cc.focal_len / camera.sensor->pixel_size_mm / camera.sensor->out_scale;
  set_exposure_rect();

  dc_gain_weight = camera.sensor->dc_gain_min_weight;
  gain_idx = camera.sensor->analog_gain_rec_idx;
  cur_ev[0] = cur_ev[1] = cur_ev[2] = get_gain_factor() * camera.sensor->sensor_analog_gains[gain_idx] * exposure_time;

  pm = std::make_unique<PubMaster>(std::vector{camera.cc.publish_name});
}

CameraState::~CameraState() {}

void CameraState::set_exposure_rect() {
  // CamX's default Bayer-histogram metering window covers 90% of the sensor
  // image. The old comma-road reference rectangle only covers the centre
  // 54% x 55% on Mi 8's 1920x1080 binned output; it therefore exposes for a
  // bright centre while leaving the rest of a road scene visibly grey/dark.
  // Use the same factory-style broad normalized window for both rear sensors.
  // This makes their exposure statistics comparable and avoids the tele camera
  // exposing from a much smaller vertical crop. Keep the old focal-length
  // rectangle available for a controlled A/B via MI8_AE_CENTER_ROI=1.
  if (camera.cc.camera_num <= 1 && getenv("MI8_AE_CENTER_ROI") == nullptr) {
    const int output_width = static_cast<int>(camera.buf.out_img_width);
    const int output_height = static_cast<int>(camera.buf.out_img_height);
    const int margin_x = output_width / 20;
    const int margin_y = output_height / 20;
    ae_xywh = (Rect){margin_x, margin_y,
                     output_width - 2 * margin_x,
                     output_height - 2 * margin_y};
    return;
  }

  // set areas for each camera, shouldn't be changed
  std::vector<std::pair<Rect, float>> ae_targets = {
    // (Rect, F)
    std::make_pair((Rect){96, 400, 1734, 524}, 567.0),  // wide
    std::make_pair((Rect){96, 160, 1734, 986}, 2648.0), // road
    std::make_pair((Rect){96, 242, 1736, 906}, 567.0)   // driver
  };
  int h_ref = 1208;
  /*
    exposure target intrinsics is
    [
      [F, 0, 0.5*ae_xywh[2]]
      [0, F, 0.5*H-ae_xywh[1]]
      [0, 0, 1]
    ]
  */
  auto ae_target = ae_targets[camera.cc.camera_num];
  Rect xywh_ref = ae_target.first;
  float fl_ref = ae_target.second;

  ae_xywh = (Rect){
    std::max(0, (int)camera.buf.out_img_width / 2 - (int)(fl_pix / fl_ref * xywh_ref.w / 2)),
    std::max(0, (int)camera.buf.out_img_height / 2 - (int)(fl_pix / fl_ref * (h_ref / 2 - xywh_ref.y))),
    std::min((int)(fl_pix / fl_ref * xywh_ref.w), (int)camera.buf.out_img_width / 2 + (int)(fl_pix / fl_ref * xywh_ref.w / 2)),
    std::min((int)(fl_pix / fl_ref * xywh_ref.h), (int)camera.buf.out_img_height / 2 + (int)(fl_pix / fl_ref * (h_ref / 2 - xywh_ref.y)))
  };
}

void CameraState::update_exposure_score(float desired_ev, int exp_t, int exp_g_idx, float exp_gain,
                                        bool ignore_transition_cost) {
  // During a tunnel exit, allowing the current gain index to penalize a large
  // downward step makes high gain (and its strong ABF profile) linger after
  // integration has already shortened. Ignore only that transition term for
  // emergency downward moves; normal AE keeps the original one-step behavior.
  const int transition_reference = ignore_transition_cost ? exp_g_idx : gain_idx;
  float score = camera.sensor->getExposureScore(desired_ev, exp_t, exp_g_idx, exp_gain,
                                                 transition_reference);
  if (score < best_ev_score) {
    new_exp_t = exp_t;
    new_exp_g = exp_g_idx;
    best_ev_score = score;
  }
}

void CameraState::set_camera_exposure(float grey_frac, int update_interval, bool fast_recovery) {
  if (!camera.enabled) return;
  std::vector<double> target_grey_minimums = {0.1, 0.1, 0.125}; // wide, road, driver

  // The dipper kernel performs costly diagnostic I2C readback for every sensor
  // update, so dual-camera AE is intentionally decimated. Preserve the
  // controller's real-time constants by accounting for skipped frames here.
  const float dt = 0.05f * update_interval;

  const float ts_grey = 10.0;
  const float ts_ev = ae_response_seconds();

  const float k_grey = (dt / ts_grey) / (1.0 + dt / ts_grey);
  const float k_ev = (dt / ts_ev) / (1.0 + dt / ts_ev);

  // It takes 3 frames for the commanded exposure settings to take effect. The first frame is already started by the time
  // we reach this function, the other 2 are due to the register buffering in the sensor.
  // Therefore we use the target EV from 3 frames ago, the grey fraction that was just measured was the result of that control action.
  // TODO: Lower latency to 2 frames, by using the histogram outputted by the sensor we can do AE before the debayering is complete

  const auto &sensor = camera.sensor;
  const bool sparse_ae = update_interval > 3;
  // Offset idx by one to not get stuck in self loop
  const float applied_ev = exposure_time * sensor->sensor_analog_gains[gain_idx] * get_gain_factor();
  const float cur_ev_ = (sparse_ae ? applied_ev : cur_ev[(camera.buf.cur_frame_data.frame_id - 1) % 3]) * sensor->ev_scale;

  // Scale target grey between min and 0.4 depending on lighting conditions
  float new_target_grey = std::clamp(0.4 - 0.3 * log2(1.0 + sensor->target_grey_factor*cur_ev_) / log2(6000.0), target_grey_minimums[camera.cc.camera_num], 0.4);
  float target_grey = (1.0 - k_grey) * target_grey_fraction + k_grey * new_target_grey;

  // The IMX363's binned V4L2 path has delayed, quantized grey statistics. Around
  // equilibrium it alternated between adjacent analog-gain bins every few frames,
  // creating visible 10% brightness flicker. Do not chase measurement noise inside
  // this bounded neutral zone; genuine lighting changes still pass through normally.
  const float grey_error = target_grey - grey_frac;
  const float measured_grey = std::max(grey_frac, 1e-4f);
  const float requested_ev = std::clamp(cur_ev_ / sensor->ev_scale * target_grey / measured_grey,
                                        sensor->min_ev, sensor->max_ev);
  float desired_ev = std::abs(grey_error) <= ae_grey_deadband()
    ? cur_ev_ / sensor->ev_scale
    : requested_ev;
  if (fast_recovery) {
    // Bright exits are one-sided and safety-critical for vision. Apply the
    // measured exposure ratio directly instead of blending it with the stale
    // dark-scene EV. The four-frame command guard below lets this command take
    // effect before another emergency correction can be sent.
    desired_ev = requested_ev;
  } else if (sparse_ae) {
    // More than three sensor frames pass between updates, so the previous
    // command is fully applied; the original three-frame delay ring no longer
    // represents the observed image.
    desired_ev = (1.0f - k_ev) * applied_ev + k_ev * desired_ev;
  } else {
    const float k = (1.0f - k_ev) / 3.0f;
    desired_ev = (k * cur_ev[0]) + (k * cur_ev[1]) + (k * cur_ev[2]) + (k_ev * desired_ev);
  }

  best_ev_score = 1e6;
  new_exp_g = 0;
  new_exp_t = 0;

  // Hysteresis around high conversion gain
  // We usually want this on since it results in lower noise, but turn off in very bright day scenes
  bool enable_dc_gain = dc_gain_enabled;
  if (!enable_dc_gain && target_grey < sensor->dc_gain_on_grey) {
    enable_dc_gain = true;
    dc_gain_weight = sensor->dc_gain_min_weight;
  } else if (enable_dc_gain && target_grey > sensor->dc_gain_off_grey) {
    enable_dc_gain = false;
    dc_gain_weight = sensor->dc_gain_max_weight;
  }

  if (enable_dc_gain && dc_gain_weight < sensor->dc_gain_max_weight) {dc_gain_weight += 1;}
  if (!enable_dc_gain && dc_gain_weight > sensor->dc_gain_min_weight) {dc_gain_weight -= 1;}

  std::string gain_bytes, time_bytes;
  if (env_ctrl_exp_from_params) {
    static Params params;
    gain_bytes = params.get("CameraDebugExpGain");
    time_bytes = params.get("CameraDebugExpTime");
  }

  int locked_gain = 0, locked_time = 0;
  const char *locked = getenv("IMX363_AE_LOCK");
  const bool lock_exposure = locked != nullptr &&
    std::sscanf(locked, "%d,%d", &locked_gain, &locked_time) == 2 &&
    locked_gain >= sensor->analog_gain_min_idx && locked_gain <= sensor->analog_gain_max_idx &&
    locked_time >= sensor->exposure_time_min && locked_time <= sensor->exposure_time_max;

  if (lock_exposure) {
    // Diagnostic override for measuring pipeline stability independently of AE.
    // It is deliberately opt-in and is never enabled in the shipped environment.
    gain_idx = locked_gain;
    exposure_time = locked_time;
    new_exp_g = gain_idx;
    new_exp_t = exposure_time;
    enable_dc_gain = false;
  } else if (gain_bytes.size() > 0 && time_bytes.size() > 0) {
    // Override gain and exposure time
    gain_idx = std::stoi(gain_bytes);
    exposure_time = std::stoi(time_bytes);

    new_exp_g = gain_idx;
    new_exp_t = exposure_time;
    enable_dc_gain = false;
  } else {
    // Simple brute force optimizer to choose sensor parameters to reach desired EV
    // Normal operation walks one gain index at a time to avoid flicker. A
    // confirmed overexposed transition may search every lower gain immediately
    // so digital gain and high-gain ABF do not keep the image soft after the
    // integration time has fallen.
    int min_g = fast_recovery ? sensor->analog_gain_min_idx
                              : std::max(gain_idx - 1, sensor->analog_gain_min_idx);
    int max_g = fast_recovery ? gain_idx
                              : std::min(gain_idx + 1, sensor->analog_gain_max_idx);
    // A mild daylight overshoot only needs a proportional EV correction. Cap
    // shutter at half a frame only for a genuinely clipped transition; doing
    // that for the 12% early-warning path would underexpose a high-gain scene.
    const bool severe_highlight = grey_frac > std::max(target_grey * 1.50f,
                                                       target_grey + 0.08f);
    const int exposure_ceiling = fast_recovery && severe_highlight
      ? std::max(sensor->exposure_time_min, sensor->exposure_time_max / 2)
      : sensor->exposure_time_max;
    for (int g = min_g; g <= max_g; g++) {
      float gain = sensor->sensor_analog_gains[g] * get_gain_factor();

      // Compute optimal time for given gain
      int t = std::clamp(int(std::round(desired_ev / gain)), sensor->exposure_time_min, exposure_ceiling);

      // Only go below recommended gain when absolutely necessary to not overexpose
      if (g < sensor->analog_gain_rec_idx && t > 20 && g < gain_idx) {
        continue;
      }

      update_exposure_score(desired_ev, t, g, gain, fast_recovery);
    }
  }

  measured_grey_fraction = grey_frac;
  target_grey_fraction = target_grey;

  analog_gain_frac = sensor->sensor_analog_gains[new_exp_g];
  if (camera.sensor->abf34_noise_lut_profiles.size() >= 2) {
    // Follow the factory ABF34 AEC regions. IMX363 currently exposes four
    // profiles; S5K3M3 exposes all seven through 64x. Hysteresis prevents a
    // small AE fluctuation from changing the DMI bank every frame.
    if (fast_recovery) {
      // A tunnel exit may drop several gain stops in one command. Select the
      // matching lower-noise table immediately instead of walking down one
      // profile per one-Hz update and leaving the recovered image visibly soft.
      constexpr float abf_thresholds[] = {2.0f, 4.0f, 8.0f, 14.0f, 32.0f, 48.0f};
      int desired_profile = 0;
      for (float threshold : abf_thresholds) {
        if (analog_gain_frac < threshold ||
            desired_profile + 1 >= static_cast<int>(camera.sensor->abf34_noise_lut_profiles.size())) break;
        ++desired_profile;
      }
      camera.ife_abf34_profile = desired_profile;
    } else if (camera.ife_abf34_profile == 0 && analog_gain_frac >= 2.0f) {
      camera.ife_abf34_profile = 1;
    } else if (camera.ife_abf34_profile == 1) {
      if (analog_gain_frac >= 4.0f && camera.sensor->abf34_noise_lut_profiles.size() >= 3) {
        camera.ife_abf34_profile = 2;
      } else if (analog_gain_frac <= 1.8f) {
        camera.ife_abf34_profile = 0;
      }
    } else if (camera.ife_abf34_profile == 2) {
      if (analog_gain_frac >= 8.0f && camera.sensor->abf34_noise_lut_profiles.size() >= 4) {
        camera.ife_abf34_profile = 3;
      } else if (analog_gain_frac <= 3.5f) {
        camera.ife_abf34_profile = 1;
      }
    } else if (camera.ife_abf34_profile == 3) {
      if (analog_gain_frac >= 14.0f && camera.sensor->abf34_noise_lut_profiles.size() >= 5) {
        camera.ife_abf34_profile = 4;
      } else if (analog_gain_frac <= 6.5f) {
        camera.ife_abf34_profile = 2;
      }
    } else if (camera.ife_abf34_profile == 4) {
      if (analog_gain_frac >= 32.0f && camera.sensor->abf34_noise_lut_profiles.size() >= 6) {
        camera.ife_abf34_profile = 5;
      } else if (analog_gain_frac <= 12.0f) {
        camera.ife_abf34_profile = 3;
      }
    } else if (camera.ife_abf34_profile == 5) {
      if (analog_gain_frac >= 48.0f && camera.sensor->abf34_noise_lut_profiles.size() >= 7) {
        camera.ife_abf34_profile = 6;
      } else if (analog_gain_frac <= 28.0f) {
        camera.ife_abf34_profile = 4;
      }
    } else if (camera.ife_abf34_profile >= 6 && analog_gain_frac <= 44.0f) {
      camera.ife_abf34_profile = 5;
    }
  }
  gain_idx = new_exp_g;
  exposure_time = new_exp_t;
  dc_gain_enabled = enable_dc_gain;

  float gain = analog_gain_frac * get_gain_factor();
  const float next_ev = exposure_time * gain;
  if (sparse_ae) {
    for (float &ev : cur_ev) ev = next_ev;
  } else {
    cur_ev[camera.buf.cur_frame_data.frame_id % 3] = next_ev;
  }

  // LOGE("ae - camera %d, cur_t %.5f, sof %.5f, dt %.5f", camera.cc.camera_num, 1e-9 * nanos_since_boot(), 1e-9 * camera.buf.cur_frame_data.timestamp_sof, 1e-9 * (nanos_since_boot() - camera.buf.cur_frame_data.timestamp_sof));

  auto exp_reg_array = sensor->getExposureRegisters(exposure_time, new_exp_g, dc_gain_enabled);
  camera.sensors_i2c(exp_reg_array.data(), exp_reg_array.size(), CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG, camera.sensor->data_word);
}

void CameraState::sendState() {
  camera.buf.sendFrameToVipc();

  MessageBuilder msg;
  auto framed = (msg.initEvent().*camera.cc.init_camera_state)();
  const FrameMetadata &meta = camera.buf.cur_frame_data;
  framed.setFrameId(meta.frame_id);
  framed.setRequestId(meta.request_id);
  framed.setTimestampEof(meta.timestamp_eof);
  framed.setTimestampSof(meta.timestamp_sof);
  framed.setIntegLines(exposure_time);
  framed.setGain(analog_gain_frac * get_gain_factor());
  framed.setHighConversionGain(dc_gain_enabled);
  framed.setMeasuredGreyFraction(measured_grey_fraction);
  framed.setTargetGreyFraction(target_grey_fraction);
  framed.setProcessingTime(meta.processing_time);

  const float ev = cur_ev[meta.frame_id % 3];
  const float perc = util::map_val(ev, camera.sensor->min_ev, camera.sensor->max_ev, 0.0f, 100.0f);
  framed.setExposureValPercent(perc);
  framed.setSensor(camera.sensor->image_sensor);

  // Log raw frames for road camera
  if (env_log_raw_frames && camera.cc.stream_type == VISION_STREAM_NARROW_ROAD && meta.frame_id % 100 == 5) {  // no overlap with qlog decimation
    framed.setImage(get_raw_frame_image(&camera.buf));
  }

  // A 2x2 median over an HD frame stalls this shared camera event loop long
  // enough for modeld to time out once per second. The 8x8 grid still samples
  // roughly 30k (wide) to 50k (tele) luma pixels across the same broad ROI,
  // while scanning 16x fewer pixels. Use it for both the one-Hz controller and
  // transition monitor so AE never delays delivery of the other camera.
  static const int exposure_interval = ae_update_interval();
  // Offset camera 1 by half an interval. The dipper CCI/debug driver can fault
  // its command DMA when both sensors submit an exposure packet together.
  const int exposure_phase = camera.cc.camera_num * (exposure_interval / 2);
  const bool regular_update = (meta.frame_id + exposure_phase) % exposure_interval == 0;
  const bool monitor_fast_exit = camera.cc.stream_type != VISION_STREAM_CABIN;

  float grey_fraction = measured_grey_fraction;
  if (regular_update || monitor_fast_exit) {
    const int x_skip = FAST_EXIT_MONITOR_SKIP;
    const int y_skip = FAST_EXIT_MONITOR_SKIP;
    grey_fraction = calculate_exposure_value(&camera.buf, ae_xywh, x_skip, y_skip);
    measured_grey_fraction = grey_fraction;
  }

  const float fast_exit_threshold = std::max(target_grey_fraction * ae_fast_exit_ratio(),
                                             target_grey_fraction + ae_fast_exit_delta());
  if (monitor_fast_exit && grey_fraction > fast_exit_threshold) {
    overexposed_frame_count = std::min(overexposed_frame_count + 1, FAST_EXIT_CONFIRM_FRAMES);
  } else {
    overexposed_frame_count = 0;
  }

  const uint64_t frames_since_update =
    last_exposure_update_frame == 0 || meta.frame_id <= last_exposure_update_frame
      ? exposure_interval : meta.frame_id - last_exposure_update_frame;
  const int fast_phase = camera.cc.camera_num * (FAST_EXIT_INTERVAL_FRAMES / 2);
  const bool fast_slot = meta.frame_id % FAST_EXIT_INTERVAL_FRAMES == static_cast<uint64_t>(fast_phase);
  const bool fast_recovery = monitor_fast_exit &&
                             overexposed_frame_count >= FAST_EXIT_CONFIRM_FRAMES &&
                             frames_since_update >= FAST_EXIT_INTERVAL_FRAMES && fast_slot;

  if (regular_update || fast_recovery) {
    const int effective_interval = fast_recovery
      ? std::clamp(static_cast<int>(frames_since_update), FAST_EXIT_INTERVAL_FRAMES, exposure_interval)
      : exposure_interval;
    set_camera_exposure(grey_fraction, effective_interval, fast_recovery);
    last_exposure_update_frame = meta.frame_id;
    if (fast_recovery) overexposed_frame_count = 0;
  }

  // Send the message
  pm->send(camera.cc.publish_name, msg);
}

void camerad_thread() {
  // TODO: centralize enabled handling

  VisionIpcServer v("camerad");

  // *** initial ISP init ***
  SpectraMaster m;
  m.init();

  // *** per-cam init ***
  std::vector<std::unique_ptr<CameraState>> cams;
  for (const auto &config : ALL_CAMERA_CONFIGS) {
    auto cam = std::make_unique<CameraState>(&m, config);
    cam->init(&v);
    cams.emplace_back(std::move(cam));
  }

  v.start_listener();

  // start devices
  LOG("-- Starting devices");
  for (auto &cam : cams) {
    // Both sensors are free-running. Starting tele one period later minus the
    // measured 20.9 ms command/SOF offset aligns it with IMX363's next SOF.
    if (cam->camera.enabled && cam->camera.cc.camera_num == 1) {
      usleep(tele_start_delay_us());
    }
    cam->camera.sensors_start();
  }

  // poll events
  LOG("-- Dequeueing Video events");
  while (!do_exit) {
    struct pollfd fds[1] = {{.fd = m.video0_fd, .events = POLLPRI}};
    int ret = poll(fds, std::size(fds), 1000);
    if (ret < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      LOGE("poll failed (%d - %d)", ret, errno);
      break;
    }

    if (!(fds[0].revents & POLLPRI)) continue;

    struct v4l2_event ev = {0};
    ret = HANDLE_EINTR(ioctl(fds[0].fd, VIDIOC_DQEVENT, &ev));
    if (ret == 0) {
      if (ev.type == V4L_EVENT_CAM_REQ_MGR_EVENT) {
        struct cam_req_mgr_message *event_data = (struct cam_req_mgr_message *)ev.u.data;
        if (env_debug_frames) {
          printf("sess_hdl 0x%6X, link_hdl 0x%6X, frame_id %lu, req_id %lu, timestamp %.2f ms, sof_status %d\n", event_data->session_hdl, event_data->u.frame_msg.link_hdl,
                 event_data->u.frame_msg.frame_id, event_data->u.frame_msg.request_id, event_data->u.frame_msg.timestamp/1e6, event_data->u.frame_msg.sof_status);
          do_exit = do_exit || event_data->u.frame_msg.frame_id > (1*20);
        }

        for (auto &cam : cams) {
          if (event_data->session_hdl == cam->camera.session_handle) {
            if (cam->camera.handle_camera_event(event_data)) {
              cam->sendState();
            }
            break;
          }
        }
      } else {
        LOGE("unhandled event %d\n", ev.type);
      }
    } else {
      LOGE("VIDIOC_DQEVENT failed, errno=%d", errno);
    }
  }
}
