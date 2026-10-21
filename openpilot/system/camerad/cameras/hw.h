#pragma once

#include <unistd.h>

#include "common/util.h"
#include "openpilot/cereal/gen/cpp/log.capnp.h"
#include "openpilot/cereal/visionstream.h"
#include "msgq/visionipc/visionipc_server.h"

#include "media/cam_isp_ife.h"


typedef enum {
  ISP_RAW_OUTPUT,   // raw frame from sensor
  ISP_IFE_PROCESSED,  // fully processed image through the IFE
  ISP_BPS_PROCESSED,  // fully processed image through the BPS
} SpectraOutputType;

// For the Xiaomi Mi 8 (dipper) / comma 3X platform

struct CameraConfig {
  int camera_num;
  VisionStreamType stream_type;
  float focal_len;  // millimeters
  const char *publish_name;
  cereal::FrameData::Builder (cereal::Event::Builder::*init_camera_state)();
  bool enabled;
  uint32_t phy;
  bool vignetting_correction;
  SpectraOutputType output_type;
};

// Xiaomi Mi 8: camera_num 0 = IMX363 rear main (CSIPHY 0).
// This is the wider of the two rear lenses, so it must feed Sunnypilot's
// WIDE_ROAD input (the same semantic mapping used by comma 3/3X).
const CameraConfig WIDE_ROAD_CAMERA_CONFIG = {
  .camera_num = 0,
  .stream_type = VISION_STREAM_WIDE_ROAD,
  .focal_len = 4.44,
  .publish_name = "wideRoadCameraState",
  .init_camera_state = &cereal::Event::Builder::initWideRoadCameraState,
  .enabled = !getenv("DISABLE_WIDE_ROAD"),
  .phy = CAM_ISP_IFE_IN_RES_PHY_0,
  .vignetting_correction = true,
  .output_type = ISP_IFE_PROCESSED,  // Use hardware ISP for demosaic+color+gamma
};

// Xiaomi Mi 8: camera_num 1 = Samsung S5K3M3SN03 rear tele (CSIPHY 1).
// The tele lens is the narrower road camera. Keep it file-gated so deleting
// one marker still falls back to the main camera's wide-only model path.
const CameraConfig ROAD_CAMERA_CONFIG = {
  .camera_num = 1,
  .stream_type = VISION_STREAM_NARROW_ROAD,
  .focal_len = 5.20,
  // Current cereal names both the narrow/tele service and transport NARROW.
  .publish_name = "narrowRoadCameraState",
  .init_camera_state = &cereal::Event::Builder::initNarrowRoadCameraState,
  .enabled = !getenv("DISABLE_ROAD") &&
             (getenv("ENABLE_TELE") != nullptr ||
              access("/data/enable_mi8_tele_camera", F_OK) == 0),
  .phy = CAM_ISP_IFE_IN_RES_PHY_1,
  // The tele now owns its Semco S5K3M3 13x17 LSC34 banks.  Keep a process
  // opt-out for a controlled A/B or emergency rollback without recompiling.
  .vignetting_correction = getenv("DISABLE_TELE_LSC") == nullptr &&
                           access("/data/disable_mi8_tele_lsc", F_OK) != 0,
  .output_type = ISP_IFE_PROCESSED,
};

// Xiaomi Mi 8: camera_num 2 = IMX576 front camera (CSIPHY 2)
// Use as DRIVER camera for driver monitoring
const CameraConfig CABIN_CAMERA_CONFIG = {
  .camera_num = 2,
  .stream_type = VISION_STREAM_CABIN,
  .focal_len = 3.67,
  .publish_name = "cabinCameraState",
  .init_camera_state = &cereal::Event::Builder::initCabinCameraState,
  .enabled = false,  // disabled for now - IMX576 driver not yet implemented
  .phy = CAM_ISP_IFE_IN_RES_PHY_2,
  .vignetting_correction = false,
  .output_type = ISP_BPS_PROCESSED,
};

const CameraConfig ALL_CAMERA_CONFIGS[] = {WIDE_ROAD_CAMERA_CONFIG, ROAD_CAMERA_CONFIG, CABIN_CAMERA_CONFIG};
