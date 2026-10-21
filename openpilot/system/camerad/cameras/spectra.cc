#include "cdm.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <stdint.h>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "media/cam_defs.h"
#include "media/cam_isp.h"
#include "media/cam_icp.h"
#include "media/cam_isp_ife.h"
#include "media/cam_sync.h"

#include "common/util.h"
#include "common/swaglog.h"
#include "system/camerad/cameras/ife.h"
#include "system/camerad/cameras/nv12_info.h"
#include "system/camerad/cameras/spectra.h"
#include "system/camerad/cameras/bps_blobs.h"


// ************** low level camera helpers ****************

int do_cam_control(int fd, int op_code, void *handle, int size) {
  struct cam_control camcontrol = {0};
  camcontrol.op_code = op_code;
  camcontrol.handle = (uint64_t)handle;
  if (size == 0) {
    camcontrol.size = 8;
    camcontrol.handle_type = CAM_HANDLE_MEM_HANDLE;
  } else {
    camcontrol.size = size;
    camcontrol.handle_type = CAM_HANDLE_USER_POINTER;
  }

  int ret = HANDLE_EINTR(ioctl(fd, VIDIOC_CAM_CONTROL, &camcontrol));
  if (ret == -1) {
    LOGE("VIDIOC_CAM_CONTROL error: op_code %d - errno %d", op_code, errno);
  }
  return ret;
}

int do_sync_control(int fd, uint32_t id, void *handle, uint32_t size) {
  struct cam_private_ioctl_arg arg = {
    .id = id,
    .size = size,
    .ioctl_ptr = (uint64_t)handle,
  };
  int ret = HANDLE_EINTR(ioctl(fd, CAM_PRIVATE_IOCTL_CMD, &arg));

  int32_t ioctl_result = static_cast<int32_t>(arg.result);
  if (ret < 0) {
    LOGE("CAM_SYNC error: id %u - errno %d - ret %d - ioctl_result %d", id, errno, ret, ioctl_result);
    return ret;
  }
  if (ioctl_result != 0) {
    LOGE("CAM_SYNC error: id %u - errno %d - ret %d - ioctl_result %d", id, errno, ret, ioctl_result);
    return ioctl_result;
  }
  return ret;
}

std::optional<int32_t> device_acquire(int fd, int32_t session_handle, void *data, uint32_t num_resources) {
  struct cam_acquire_dev_cmd cmd = {
    .session_handle = session_handle,
    .handle_type = CAM_HANDLE_USER_POINTER,
    .num_resources = (uint32_t)(data ? num_resources : 0),
    .resource_hdl = (uint64_t)data,
  };
  int err = do_cam_control(fd, CAM_ACQUIRE_DEV, &cmd, sizeof(cmd));
  return err == 0 ? std::make_optional(cmd.dev_handle) : std::nullopt;
}

int device_config(int fd, int32_t session_handle, int32_t dev_handle, uint64_t packet_handle) {
  struct cam_config_dev_cmd cmd = {
    .session_handle = session_handle,
    .dev_handle = dev_handle,
    .packet_handle = packet_handle,
  };
  return do_cam_control(fd, CAM_CONFIG_DEV, &cmd, sizeof(cmd));
}

int device_control(int fd, int op_code, int session_handle, int dev_handle) {
  // start stop and release are all the same
  struct cam_start_stop_dev_cmd cmd { .session_handle = session_handle, .dev_handle = dev_handle };
  return do_cam_control(fd, op_code, &cmd, sizeof(cmd));
}

void *alloc_w_mmu_hdl(int video0_fd, int len, uint32_t *handle, int align, int flags, int mmu_hdl, int mmu_hdl2) {
  struct cam_mem_mgr_alloc_cmd mem_mgr_alloc_cmd = {0};
  mem_mgr_alloc_cmd.len = len;
  mem_mgr_alloc_cmd.align = align;
  mem_mgr_alloc_cmd.flags = flags;
  mem_mgr_alloc_cmd.num_hdl = 0;
  if (mmu_hdl != 0) {
    mem_mgr_alloc_cmd.mmu_hdls[0] = mmu_hdl;
    mem_mgr_alloc_cmd.num_hdl++;
  }
  if (mmu_hdl2 != 0) {
    mem_mgr_alloc_cmd.mmu_hdls[1] = mmu_hdl2;
    mem_mgr_alloc_cmd.num_hdl++;
  }

  do_cam_control(video0_fd, CAM_REQ_MGR_ALLOC_BUF, &mem_mgr_alloc_cmd, sizeof(mem_mgr_alloc_cmd));
  *handle = mem_mgr_alloc_cmd.out.buf_handle;

  void *ptr = NULL;
  if (mem_mgr_alloc_cmd.out.fd > 0) {
    ptr = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, mem_mgr_alloc_cmd.out.fd, 0);
    assert(ptr != MAP_FAILED);
  }

  // LOGD("allocated: %x %d %llx mapped %p", mem_mgr_alloc_cmd.out.buf_handle, mem_mgr_alloc_cmd.out.fd, mem_mgr_alloc_cmd.out.vaddr, ptr);

  return ptr;
}

void release(int video0_fd, uint32_t handle) {
  struct cam_mem_mgr_release_cmd mem_mgr_release_cmd = {0};
  mem_mgr_release_cmd.buf_handle = handle;

  int ret = do_cam_control(video0_fd, CAM_REQ_MGR_RELEASE_BUF, &mem_mgr_release_cmd, sizeof(mem_mgr_release_cmd));
  assert(ret == 0);
}

static cam_cmd_power *power_set_wait(cam_cmd_power *power, int16_t delay_ms) {
  cam_cmd_unconditional_wait *unconditional_wait = (cam_cmd_unconditional_wait *)((char *)power + (sizeof(struct cam_cmd_power) + (power->count - 1) * sizeof(struct cam_power_settings)));
  unconditional_wait->cmd_type = CAMERA_SENSOR_CMD_TYPE_WAIT;
  unconditional_wait->delay = delay_ms;
  unconditional_wait->op_code = CAMERA_SENSOR_WAIT_OP_SW_UCND;
  return (struct cam_cmd_power *)(unconditional_wait + 1);
}

// *** MemoryManager ***

void *MemoryManager::alloc_buf(int size, uint32_t *handle) {
  void *ptr;
  auto &cache = cached_allocations[size];
  if (!cache.empty()) {
    ptr = cache.front();
    cache.pop();
    *handle = handle_lookup[ptr];
  } else {
    ptr = alloc_w_mmu_hdl(video0_fd, size, handle);
    handle_lookup[ptr] = *handle;
    size_lookup[ptr] = size;
  }
  memset(ptr, 0, size);
  return ptr;
}

void MemoryManager::free(void *ptr) {
  cached_allocations[size_lookup[ptr]].push(ptr);
}

MemoryManager::~MemoryManager() {
  for (auto& x : cached_allocations) {
    while (!x.second.empty()) {
      void *ptr = x.second.front();
      x.second.pop();
      LOGD("freeing cached allocation %p with size %d", ptr, size_lookup[ptr]);
      munmap(ptr, size_lookup[ptr]);

      // release fd
      close(handle_lookup[ptr] >> 16);
      release(video0_fd, handle_lookup[ptr]);

      handle_lookup.erase(ptr);
      size_lookup.erase(ptr);
    }
  }
}

// *** SpectraMaster ***

void SpectraMaster::init() {
  LOG("-- Opening devices");
  // video0 is req_mgr, the target of many ioctls
  video0_fd = HANDLE_EINTR(open("/dev/v4l/by-path/platform-soc:qcom_cam-req-mgr-video-index0", O_RDWR | O_NONBLOCK));
  assert(video0_fd >= 0);
  LOGD("opened video0");

  // video1 is cam_sync, the target of some ioctls
  cam_sync_fd = HANDLE_EINTR(open("/dev/v4l/by-path/platform-cam_sync-video-index0", O_RDWR | O_NONBLOCK));
  assert(cam_sync_fd >= 0);
  LOGD("opened video1 (cam_sync)");

  // looks like there's only one of these
  isp_fd = open_v4l_by_name_and_index("cam-isp");
  assert(isp_fd >= 0);
  LOGD("opened isp %d", (int)isp_fd);

  icp_fd = open_v4l_by_name_and_index("cam-icp");
  assert(icp_fd >= 0);
  LOGD("opened icp %d", (int)icp_fd);

  // query ISP for MMU handles
  LOG("-- Query for MMU handles");
  struct cam_isp_query_cap_cmd isp_query_cap_cmd = {0};
  struct cam_query_cap_cmd query_cap_cmd = {0};
  query_cap_cmd.handle_type = 1;
  query_cap_cmd.caps_handle = (uint64_t)&isp_query_cap_cmd;
  query_cap_cmd.size = sizeof(isp_query_cap_cmd);
  int ret = do_cam_control(isp_fd, CAM_QUERY_CAP, &query_cap_cmd, sizeof(query_cap_cmd));
  assert(ret == 0);
  LOGD("using MMU handle: %x", isp_query_cap_cmd.device_iommu.non_secure);
  LOGD("using MMU handle: %x", isp_query_cap_cmd.cdm_iommu.non_secure);
  device_iommu = isp_query_cap_cmd.device_iommu.non_secure;
  cdm_iommu = isp_query_cap_cmd.cdm_iommu.non_secure;

  // query ICP for MMU handles
  struct cam_icp_query_cap_cmd icp_query_cap_cmd = {0};
  query_cap_cmd.caps_handle = (uint64_t)&icp_query_cap_cmd;
  query_cap_cmd.size = sizeof(icp_query_cap_cmd);
  ret = do_cam_control(icp_fd, CAM_QUERY_CAP, &query_cap_cmd, sizeof(query_cap_cmd));
  assert(ret == 0);
  LOGD("using ICP MMU handle: %x", icp_query_cap_cmd.dev_iommu_handle.non_secure);
  icp_device_iommu = icp_query_cap_cmd.dev_iommu_handle.non_secure;

  // subscribe
  LOG("-- Subscribing");
  struct v4l2_event_subscription sub = {0};
  sub.type = V4L_EVENT_CAM_REQ_MGR_EVENT;
  sub.id = V4L_EVENT_CAM_REQ_MGR_SOF_BOOT_TS;
  ret = HANDLE_EINTR(ioctl(video0_fd, VIDIOC_SUBSCRIBE_EVENT, &sub));
  LOGD("req mgr subscribe: %d", ret);

  mem_mgr.init(video0_fd);
}

// *** SpectraCamera ***

SpectraCamera::SpectraCamera(SpectraMaster *master, const CameraConfig &config)
  : m(master),
    enabled(config.enabled),
    cc(config) {
  ife_buf_depth = VIPC_BUFFER_COUNT;
  assert(ife_buf_depth < MAX_IFE_BUFS);
}

SpectraCamera::~SpectraCamera() {
  if (open) {
    camera_close();
  }
}

int SpectraCamera::clear_req_queue() {
  // for "non-realtime" BPS
  if (icp_dev_handle > 0) {
    struct cam_flush_dev_cmd cmd = {
      .session_handle = session_handle,
      .dev_handle = icp_dev_handle,
      .flush_type = CAM_FLUSH_TYPE_ALL,
    };
    int err = do_cam_control(m->icp_fd, CAM_FLUSH_REQ, &cmd, sizeof(cmd));
    assert(err == 0);
    LOGD("flushed bps: %d", err);
  }

  // for "realtime" devices
  struct cam_req_mgr_flush_info req_mgr_flush_request = {0};
  req_mgr_flush_request.session_hdl = session_handle;
  req_mgr_flush_request.link_hdl = link_handle;
  req_mgr_flush_request.flush_type = CAM_REQ_MGR_FLUSH_TYPE_ALL;
  int ret = do_cam_control(m->video0_fd, CAM_REQ_MGR_FLUSH_REQ, &req_mgr_flush_request, sizeof(req_mgr_flush_request));
  LOGD("flushed all req: %d", ret);  // returns a "time until timeout" on clearing the workq

  for (int i = 0; i < MAX_IFE_BUFS; ++i) {
    destroySyncObjectAt(i);
  }

  return ret;
}

void SpectraCamera::camera_open(VisionIpcServer *v) {
  // Bail out before touching hardware: openSensor() probes and acquires the sensor, and on
  // xiaomi8 several configs share camera_num 0 (only ROAD is enabled), so probing for a
  // disabled camera would re-acquire the sensor ROAD already holds and abort on
  // assert(sensor_dev_handle_). comma3 hides this because each config has its own sensor.
  if (!enabled) return;

  if (!openSensor()) {
    return;
  }

  if (!enabled) return;

  buf.out_img_width = sensor->frame_width / sensor->out_scale;
  buf.out_img_height = (sensor->hdr_offset > 0 ? (sensor->frame_height - sensor->hdr_offset) / 2 : sensor->frame_height) / sensor->out_scale;

  // size is driven by all the HW that handles frames,
  // the video encoder has certain alignment requirements in this case
  std::tie(stride, y_height, uv_height, yuv_size) = get_nv12_info(buf.out_img_width, buf.out_img_height);
  uv_offset = stride * y_height;

  open = true;
  configISP();
  if (cc.output_type == ISP_BPS_PROCESSED) configICP();
  configCSIPHY();
  linkDevices();

  LOGD("camera init %d", cc.camera_num);
  buf.init(nullptr, nullptr, this, v, ife_buf_depth, cc.stream_type);  // 2026-08-30: ISP_IFE_PROCESSED path, no GPU demosaic needed
  camera_map_bufs();
  clearAndRequeue(1);
}

void SpectraCamera::sensors_start() {
  if (!enabled) return;
  LOGD("starting sensor %d", cc.camera_num);
  sensors_i2c(sensor->start_reg_array.data(), sensor->start_reg_array.size(), CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG, sensor->data_word);
}

void SpectraCamera::sensors_poke(int request_id) {
  uint32_t cam_packet_handle = 0;
  // The kernel rejects a buffer exactly the size of the packet: cam_sensor_i2c_pkt_parse()
  // tests `config.offset >= len_of_buff - sizeof(struct cam_packet)`, which is 0 >= 0 for a
  // 64-byte allocation, so every poke failed with "Failed CCI Config: -22". Pad it out.
  int size = sizeof(struct cam_packet) + 8;
  auto pkt = m->mem_mgr.alloc<struct cam_packet>(size, &cam_packet_handle);
  pkt->num_cmd_buf = 0;
  pkt->kmd_cmd_buf_index = -1;
  pkt->header.size = size;
  pkt->header.op_code = CAM_SENSOR_PACKET_OPCODE_SENSOR_NOP;
  pkt->header.request_id = request_id;

  int ret = device_config(sensor_fd, session_handle, sensor_dev_handle, cam_packet_handle);
  if (ret != 0) {
    LOGE("** sensor %d FAILED poke, disabling", cc.camera_num);
    enabled = false;
    return;
  }
}

void SpectraCamera::sensors_i2c(const struct i2c_random_wr_payload* dat, int len, int op_code, bool data_word) {
  // LOGD("sensors_i2c: %d", len);
  uint32_t cam_packet_handle = 0;
  int size = sizeof(struct cam_packet)+sizeof(struct cam_cmd_buf_desc)*1;
  auto pkt = m->mem_mgr.alloc<struct cam_packet>(size, &cam_packet_handle);
  pkt->num_cmd_buf = 1;
  pkt->kmd_cmd_buf_index = -1;
  pkt->header.size = size;
  pkt->header.op_code = op_code;
  struct cam_cmd_buf_desc *buf_desc = (struct cam_cmd_buf_desc *)&pkt->payload;

  buf_desc[0].size = buf_desc[0].length = sizeof(struct i2c_rdwr_header) + len*sizeof(struct i2c_random_wr_payload);
  buf_desc[0].type = CAM_CMD_BUF_I2C;

  auto i2c_random_wr = m->mem_mgr.alloc<struct cam_cmd_i2c_random_wr>(buf_desc[0].size, (uint32_t*)&buf_desc[0].mem_handle);
  i2c_random_wr->header.count = len;
  i2c_random_wr->header.op_code = 1;
  i2c_random_wr->header.cmd_type = CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR;
  i2c_random_wr->header.data_type = data_word ? CAMERA_SENSOR_I2C_TYPE_WORD : CAMERA_SENSOR_I2C_TYPE_BYTE;
  i2c_random_wr->header.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  memcpy(i2c_random_wr->random_wr_payload, dat, len*sizeof(struct i2c_random_wr_payload));

  int ret = device_config(sensor_fd, session_handle, sensor_dev_handle, cam_packet_handle);
  if (ret != 0) {
    LOGE("** sensor %d FAILED i2c, disabling", cc.camera_num);
    enabled = false;
    return;
  }
}

int SpectraCamera::sensors_init() {
  uint32_t cam_packet_handle = 0;
  int size = sizeof(struct cam_packet)+sizeof(struct cam_cmd_buf_desc)*2;
  auto pkt = m->mem_mgr.alloc<struct cam_packet>(size, &cam_packet_handle);
  pkt->num_cmd_buf = 2;
  pkt->kmd_cmd_buf_index = -1;
  pkt->header.op_code = CSLDeviceTypeImageSensor | CAM_SENSOR_PACKET_OPCODE_SENSOR_PROBE;
  pkt->header.size = size;
  struct cam_cmd_buf_desc *buf_desc = (struct cam_cmd_buf_desc *)&pkt->payload;

  buf_desc[0].size = buf_desc[0].length = sizeof(struct cam_cmd_i2c_info) + sizeof(struct cam_cmd_probe);
  buf_desc[0].type = CAM_CMD_BUF_LEGACY;
  auto i2c_info = m->mem_mgr.alloc<struct cam_cmd_i2c_info>(buf_desc[0].size, (uint32_t*)&buf_desc[0].mem_handle);
  auto probe = (struct cam_cmd_probe *)(i2c_info.get() + 1);

  probe->camera_id = cc.camera_num;
  i2c_info->slave_addr = sensor->getSlaveAddress(cc.camera_num);
  // 0(I2C_STANDARD_MODE) = 100khz, 1(I2C_FAST_MODE) = 400khz
  //i2c_info->i2c_freq_mode = I2C_STANDARD_MODE;
  i2c_info->i2c_freq_mode = I2C_FAST_MODE;
  i2c_info->cmd_type = CAMERA_SENSOR_CMD_TYPE_I2C_INFO;

  probe->data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  probe->addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  probe->op_code = 3;   // don't care?
  probe->cmd_type = CAMERA_SENSOR_CMD_TYPE_PROBE;
  probe->reg_addr = sensor->probe_reg_addr;
  probe->expected_data = sensor->probe_expected_data;
  probe->data_mask = 0;

  // Allocate generously; the exact byte count is computed after the sequence is built
  // (it used to be hardcoded at 196, which silently encoded the old 4-entry power-up group).
  buf_desc[1].size = buf_desc[1].length = 256;
  buf_desc[1].type = CAM_CMD_BUF_I2C;
  auto power_settings = m->mem_mgr.alloc<struct cam_cmd_power>(buf_desc[1].size, (uint32_t*)&buf_desc[1].mem_handle);

  struct cam_cmd_power *power = power_settings.get();
  if (cc.camera_num == 1) {
    // Xiaomi's stock S5K3M3SN03 sequence, decoded from the vendor sensor bin:
    // VANA -> VIO -> VDIG, RESET high, then 24 MHz MCLK. Unlike IMX363 this
    // module has no CUSTOM_GPIO1 rail.
    power->count = 3;
    power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_UP;
    power->power_settings[0].power_seq_type = 1;  // VANA
    power->power_settings[1].power_seq_type = 3;  // VIO
    power->power_settings[2].power_seq_type = 2;  // VDIG
    power = power_set_wait(power, 1);

    power->count = 1;
    power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_UP;
    power->power_settings[0].power_seq_type = 8;  // RESET high
    power->power_settings[0].config_val_low = 1;
    power = power_set_wait(power, 12);

    power->count = 1;
    power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_UP;
    power->power_settings[0].power_seq_type = 0;  // MCLK
    power->power_settings[0].config_val_low = sensor->mclk_frequency;
    power = power_set_wait(power, 10);

    // The vendor down sequence uses one common 1 ms delay after the group.
    power->count = 5;
    power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_DOWN;
    power->power_settings[0].power_seq_type = 8;  // RESET low
    power->power_settings[0].config_val_low = 0;
    power->power_settings[1].power_seq_type = 2;  // VDIG
    power->power_settings[2].power_seq_type = 3;  // VIO
    power->power_settings[3].power_seq_type = 1;  // VANA
    power->power_settings[4].power_seq_type = 0;  // MCLK off
    power->power_settings[4].config_val_low = 0;
    power = power_set_wait(power, 1);
  } else {
  // power on
  // Power seq types are the kernel's msm_camera_power_seq_type (cam_sensor_cmn_header.h,
  // not exported through UAPI): 0=MCLK 1=VANA 2=VDIG 3=VIO 4=VAF 8=RESET 10=CUSTOM_GPIO1.
  power->count = 5;
  power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_UP;
  power->power_settings[0].power_seq_type = 3;  // VIO
  power->power_settings[1].power_seq_type = 1;  // VANA (analog)
  power->power_settings[2].power_seq_type = 2;  // VDIG (digital)
  // ** xiaomi8 (dipper) / IMX363 **: this sensor does not come up without CUSTOM_GPIO1
  // (device tree `gpio-custom1` -> GPIO 40 / CAM_CUSTOM0) asserted. Omitting it leaves the
  // sensor unpowered, CCI reads return 0 words, and probe fails "chip id 0 does not match 363".
  // comma3's OX03C10/OS04C10 have no such GPIO; the kernel's msm_cam_sensor_handle_reg_gpio()
  // silently skips a seq type whose gpio_num_info->valid[] is 0, so this is a no-op there.
  power->power_settings[3].power_seq_type = 10;  // CUSTOM_GPIO1
  power->power_settings[3].config_val_low = 1;
  power->power_settings[4].power_seq_type = 8;  // reset low
  power = power_set_wait(power, 1);

  // set clock
  power->count = 1;
  power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_UP;
  power->power_settings[0].power_seq_type = 0;
  power->power_settings[0].config_val_low = sensor->mclk_frequency;
  power = power_set_wait(power, 1);

  // reset high
  power->count = 1;
  power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_UP;
  power->power_settings[0].power_seq_type = 8;
  power->power_settings[0].config_val_low = 1;
  // wait 650000 cycles @ 19.2 mhz = 33.8 ms
  power = power_set_wait(power, 34);

  // probe happens here

  // disable clock
  power->count = 1;
  power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_DOWN;
  power->power_settings[0].power_seq_type = 0;
  power->power_settings[0].config_val_low = 0;
  power = power_set_wait(power, 1);

  // reset high
  power->count = 1;
  power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_DOWN;
  power->power_settings[0].power_seq_type = 8;
  power->power_settings[0].config_val_low = 1;
  power = power_set_wait(power, 1);

  // reset low
  power->count = 1;
  power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_DOWN;
  power->power_settings[0].power_seq_type = 8;
  power->power_settings[0].config_val_low = 0;
  power = power_set_wait(power, 1);

  // power off (mirror of the power-up group, including CUSTOM_GPIO1)
  power->count = 4;
  power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_DOWN;
  power->power_settings[0].power_seq_type = 10;  // CUSTOM_GPIO1
  power->power_settings[0].config_val_low = 0;
  power->power_settings[1].power_seq_type = 2;   // VDIG
  power->power_settings[2].power_seq_type = 1;   // VANA
  power->power_settings[3].power_seq_type = 3;   // VIO

  // The last group carries no trailing wait, so step over it by hand to get the exact
  // byte count the kernel should parse.
  power = (struct cam_cmd_power *)((char *)power + sizeof(struct cam_cmd_power) +
                                   (power->count - 1) * sizeof(struct cam_power_settings));
  }
  buf_desc[1].size = buf_desc[1].length = (uint32_t)((char *)power - (char *)power_settings.get());
  assert(buf_desc[1].length <= 256);

  int ret = do_cam_control(sensor_fd, CAM_SENSOR_PROBE_CMD, (void *)(uintptr_t)cam_packet_handle, 0);
  LOGD("probing the sensor: %d", ret);
  return ret;
}

void add_patch(struct cam_packet *pkt, int32_t dst_hdl, uint32_t dst_offset, int32_t src_hdl, uint32_t src_offset) {
  void *ptr = (char*)&pkt->payload + pkt->patch_offset;
  struct cam_patch_desc *p = (struct cam_patch_desc *)((unsigned char*)ptr + sizeof(struct cam_patch_desc)*pkt->num_patches);
  p->dst_buf_hdl = dst_hdl;
  p->src_buf_hdl = src_hdl;
  p->dst_offset = dst_offset;
  p->src_offset = src_offset;
  pkt->num_patches++;
};

void SpectraCamera::config_bps(int idx, int request_id) {
  /*
    Handles per-frame BPS config.
    * BPS = Bayer Processing Segment
  */

  bool needs_downscale = sensor->out_scale > 1;
  int num_io_cfgs = needs_downscale ? 3 : 2;
  int num_patches = needs_downscale ? 14 : 12;
  int size = sizeof(struct cam_packet) + sizeof(struct cam_cmd_buf_desc)*2 + sizeof(struct cam_buf_io_cfg)*num_io_cfgs;
  size += sizeof(struct cam_patch_desc)*num_patches;

  uint32_t cam_packet_handle = 0;
  auto pkt = m->mem_mgr.alloc<struct cam_packet>(size, &cam_packet_handle);

  pkt->header.op_code = CSLDeviceTypeBPS | CAM_ICP_OPCODE_BPS_UPDATE;
  pkt->header.request_id = request_id;
  pkt->header.size = size;

  typedef struct {
    struct {
      uint32_t ptr[2];
      uint32_t unknown[2];
    } frames[9];

    uint32_t unknown1;
    uint32_t unknown2;
    uint32_t unknown3;
    uint32_t unknown4;

    uint32_t cdm_addr;
    uint32_t cdm_size;
    uint32_t settings_addr;
    uint32_t striping_addr;
    uint32_t cdm_addr2;

    uint32_t req_id;
    uint64_t handle;
  } bps_tmp;

  typedef struct {
    uint32_t a;
    uint32_t n;
    unsigned base : 32;
    unsigned unused : 12;
    unsigned length : 20;
    uint32_t p;
    uint32_t u;
    uint32_t h;
    uint32_t b;
  } cdm_tmp;

  // *** cmd buf ***
  std::vector<uint32_t> patches;
  struct cam_cmd_buf_desc *buf_desc = (struct cam_cmd_buf_desc *)&pkt->payload;
  {
    pkt->num_cmd_buf = 2;
    pkt->kmd_cmd_buf_index = -1;
    pkt->kmd_cmd_buf_offset = 0;

    buf_desc[0].meta_data = 0;
    buf_desc[0].mem_handle = bps_cmd.handle;
    buf_desc[0].type = CAM_CMD_BUF_FW;
    buf_desc[0].offset = bps_cmd.aligned_size()*idx;

    buf_desc[0].length = sizeof(bps_tmp) + sizeof(cdm_tmp);
    buf_desc[0].size = buf_desc[0].length;

    // rest gets patched in
    bps_tmp *fp = (bps_tmp *)((unsigned char *)bps_cmd.ptr + buf_desc[0].offset);
    memset(fp, 0, buf_desc[0].length);
    fp->handle = (uint64_t)icp_dev_handle;
    fp->cdm_size = bps_cdm_striping_bl.size;   // this comes from the striping lib create call
    fp->req_id = 0; // why always 0?

    cdm_tmp *pa = (cdm_tmp *)((unsigned char *)fp + sizeof(bps_tmp));
    pa->a = 0;
    pa->n = 1;
    pa->p = 20;  // GENERIC
    pa->u = 0;
    pa->h = 0;
    pa->b = 0;
    pa->unused = 0;
    pa->base = 0; // this gets patched

    int cdm_len = 0;

    if (bps_lin_reg.size() == 0) {
      // set first knee pt to do BLC
      uint32_t new_knee[8];
      new_knee[0] = sensor->black_level << (14 - sensor->bits_per_pixel);
      for (int i = 0; i < 7; i++) {
        uint32_t pts = sensor->linearization_pts[i / 2];
        new_knee[i + 1] = (i % 2 == 0) ? (pts >> 16) : (pts & 0xffff);
      }
      for (int i = 0; i < 4; i++) {
        bps_lin_reg.push_back((new_knee[2*i + 1] << 16) | new_knee[2*i]);
      }
    }

    if (bps_ccm_reg.size() == 0) {
      for (int i = 0; i < 3; i++) {
        bps_ccm_reg.push_back(sensor->color_correct_matrix[i] | (sensor->color_correct_matrix[i+3] << 0x10));
        bps_ccm_reg.push_back(sensor->color_correct_matrix[i+6]);
      }
    }

    // white balance
    cdm_len += write_cont((unsigned char *)bps_cdm_program_array.ptr + cdm_len, 0x2868, {
      0x04000400,
      0x00000400,
      0x00000000,
      0x00000000,
    });
    // debayer
    cdm_len += write_cont((unsigned char *)bps_cdm_program_array.ptr + cdm_len, 0x2878, {
      0x00000080,
      0x00800066,
    });
    // linearization
    cdm_len += write_cont((unsigned char *)bps_cdm_program_array.ptr + cdm_len, 0x1868, bps_lin_reg);
    cdm_len += write_cont((unsigned char *)bps_cdm_program_array.ptr + cdm_len, 0x1878, bps_lin_reg);
    cdm_len += write_cont((unsigned char *)bps_cdm_program_array.ptr + cdm_len, 0x1888, bps_lin_reg);
    cdm_len += write_cont((unsigned char *)bps_cdm_program_array.ptr + cdm_len, 0x1898, bps_lin_reg);
    uint64_t addr;
    cdm_len += write_dmi((unsigned char *)bps_cdm_program_array.ptr + cdm_len, &addr, sensor->linearization_lut.size()*sizeof(uint32_t), 0x1808, 1, CAM_CDM_CMD_DMI);
    patches.push_back(addr - (uint64_t)bps_cdm_program_array.ptr);

    // color correction
    cdm_len += write_cont((unsigned char *)bps_cdm_program_array.ptr + cdm_len, 0x2e68, bps_ccm_reg);

    // gamma
    for (uint8_t ch = 1; ch <= 3; ch++) {
      cdm_len += write_dmi((unsigned char *)bps_cdm_program_array.ptr + cdm_len, &addr, sensor->gamma_lut_rgb.size()*sizeof(uint32_t), 0x3208, ch, CAM_CDM_CMD_DMI);
      patches.push_back(addr - (uint64_t)bps_cdm_program_array.ptr);
    }

    cdm_len += build_common_ife_bps((unsigned char *)bps_cdm_program_array.ptr + cdm_len, cc, sensor.get(), patches, false);

    pa->length = cdm_len - 1;

    // *** second command ***
    // parsed by cam_icp_packet_generic_blob_handler
    struct isp_packet {
      uint32_t header;
      struct cam_icp_clk_bw_request clk;
    } __attribute__((packed)) tmp;
    tmp.header = CAM_ICP_CMD_GENERIC_BLOB_CLK;
    tmp.header |= (sizeof(cam_icp_clk_bw_request)) << 8;
    tmp.clk.budget_ns = 0x1fca058;
    tmp.clk.frame_cycles = sensor->frame_width * sensor->frame_height; // matches striping lib pixelCount
    tmp.clk.rt_flag = 0x0;
    tmp.clk.uncompressed_bw = 0x38512180;
    tmp.clk.compressed_bw = 0x38512180;

    buf_desc[1].size = sizeof(tmp);
    buf_desc[1].offset = 0;
    buf_desc[1].length = buf_desc[1].size - buf_desc[1].offset;
    buf_desc[1].type = CAM_CMD_BUF_GENERIC;
    buf_desc[1].meta_data = CAM_ICP_CMD_META_GENERIC_BLOB;
    auto buf2 = m->mem_mgr.alloc<uint32_t>(buf_desc[1].size, (uint32_t*)&buf_desc[1].mem_handle);
    memcpy(buf2.get(), &tmp, sizeof(tmp));
  }

  // *** io config ***
  pkt->num_io_configs = num_io_cfgs;
  pkt->io_configs_offset = sizeof(struct cam_cmd_buf_desc)*pkt->num_cmd_buf;
  struct cam_buf_io_cfg *io_cfg = (struct cam_buf_io_cfg *)((char*)&pkt->payload + pkt->io_configs_offset);
  {
    // input frame
    io_cfg[0].offsets[0] = 0;
    io_cfg[0].mem_handle[0] = buf_handle_raw[idx];

    io_cfg[0].planes[0] = (struct cam_plane_cfg){
      .width = sensor->frame_width,
      .height = sensor->frame_height + sensor->extra_height,
      .plane_stride = sensor->frame_stride,
      .slice_height = sensor->frame_height + sensor->extra_height,
    };
    io_cfg[0].format = sensor->mipi_format;
    io_cfg[0].color_space = CAM_COLOR_SPACE_BASE;
    io_cfg[0].color_pattern = 0x5;
    io_cfg[0].bpp = (sensor->mipi_format == CAM_FORMAT_MIPI_RAW_10 ? 0xa : 0xc);
    io_cfg[0].resource_type = CAM_ICP_BPS_INPUT_IMAGE;
    io_cfg[0].fence = sync_objs_ife[idx];
    io_cfg[0].direction = CAM_BUF_INPUT;
    io_cfg[0].subsample_pattern = 0x1;
    io_cfg[0].framedrop_pattern = 0x1;

    // output frame
    io_cfg[1].mem_handle[0] = buf_handle_yuv[idx];
    io_cfg[1].mem_handle[1] = buf_handle_yuv[idx];
    io_cfg[1].planes[0] = (struct cam_plane_cfg){
      .width = buf.out_img_width,
      .height = buf.out_img_height,
      .plane_stride = stride,
      .slice_height = y_height,
    };
    io_cfg[1].planes[1] = (struct cam_plane_cfg){
      .width = buf.out_img_width,
      .height = buf.out_img_height / 2,
      .plane_stride = stride,
      .slice_height = uv_height,
    };
    io_cfg[1].offsets[1] = ALIGNED_SIZE(io_cfg[1].planes[0].plane_stride*io_cfg[1].planes[0].slice_height, 0x1000);
    assert(io_cfg[1].offsets[1] == uv_offset);

    io_cfg[1].format = CAM_FORMAT_NV12;  // TODO: why is this 21 in the dump? should be 12
    io_cfg[1].color_space = CAM_COLOR_SPACE_BT601_FULL;
    io_cfg[1].resource_type = needs_downscale ? CAM_ICP_BPS_OUTPUT_IMAGE_REG1 : CAM_ICP_BPS_OUTPUT_IMAGE_FULL;
    io_cfg[1].fence = sync_objs_bps[idx];
    io_cfg[1].direction = CAM_BUF_OUTPUT;
    io_cfg[1].subsample_pattern = 0x1;
    io_cfg[1].framedrop_pattern = 0x1;

    if (needs_downscale) {
      // downscaling needs a full res placeholder
      uint32_t full_stride, full_y_h, full_uv_h, full_yuv_size;
      std::tie(full_stride, full_y_h, full_uv_h, full_yuv_size) = get_nv12_info(sensor->frame_width, sensor->frame_height);
      io_cfg[2].mem_handle[0] = bps_fullres_dummy.handle;
      io_cfg[2].mem_handle[1] = bps_fullres_dummy.handle;
      io_cfg[2].planes[0] = (struct cam_plane_cfg){
        .width = sensor->frame_width,
        .height = sensor->frame_height,
        .plane_stride = full_stride,
        .slice_height = full_y_h,
      };
      io_cfg[2].planes[1] = (struct cam_plane_cfg){
        .width = sensor->frame_width,
        .height = sensor->frame_height / 2,
        .plane_stride = full_stride,
        .slice_height = full_uv_h,
      };
      io_cfg[2].offsets[1] = ALIGNED_SIZE(full_stride * full_y_h, 0x1000);
      io_cfg[2].format = CAM_FORMAT_NV12;
      io_cfg[2].color_space = CAM_COLOR_SPACE_BT601_FULL;
      io_cfg[2].resource_type = CAM_ICP_BPS_OUTPUT_IMAGE_FULL;
      io_cfg[2].fence = sync_objs_bps[idx];
      io_cfg[2].direction = CAM_BUF_OUTPUT;
      io_cfg[2].subsample_pattern = 0x1;
      io_cfg[2].framedrop_pattern = 0x1;
    }
  }

  // *** patches ***
  {
    assert(patches.size() == 0 || patches.size() == 4);
    pkt->patch_offset = sizeof(struct cam_cmd_buf_desc)*pkt->num_cmd_buf + sizeof(struct cam_buf_io_cfg)*pkt->num_io_configs;

    if (patches.size() > 0) {
      // linearization LUT
      add_patch(pkt.get(), bps_cdm_program_array.handle, patches[0], bps_linearization_lut.handle, 0);
      // gamma LUTs
      for (int i = 0; i < 3; i++) {
        add_patch(pkt.get(), bps_cdm_program_array.handle, patches[i+1], bps_gamma_lut.handle, 0);
      }
    }

    // input frame
    add_patch(pkt.get(), bps_cmd.handle, buf_desc[0].offset + offsetof(bps_tmp, frames[0].ptr[0]), buf_handle_raw[idx], 0);

    if (needs_downscale) {
      add_patch(pkt.get(), bps_cmd.handle, buf_desc[0].offset + offsetof(bps_tmp, frames[1].ptr[0]), bps_fullres_dummy.handle, 0);
      add_patch(pkt.get(), bps_cmd.handle, buf_desc[0].offset + offsetof(bps_tmp, frames[1].ptr[1]), bps_fullres_dummy.handle, io_cfg[2].offsets[1]);
      // output frame at REG1
      add_patch(pkt.get(), bps_cmd.handle, buf_desc[0].offset + offsetof(bps_tmp, frames[7].ptr[0]), buf_handle_yuv[idx], 0);
      add_patch(pkt.get(), bps_cmd.handle, buf_desc[0].offset + offsetof(bps_tmp, frames[7].ptr[1]), buf_handle_yuv[idx], io_cfg[1].offsets[1]);
    } else {
      // output frame at FULL
      add_patch(pkt.get(), bps_cmd.handle, buf_desc[0].offset + offsetof(bps_tmp, frames[1].ptr[0]), buf_handle_yuv[idx], 0);
      add_patch(pkt.get(), bps_cmd.handle, buf_desc[0].offset + offsetof(bps_tmp, frames[1].ptr[1]), buf_handle_yuv[idx], io_cfg[1].offsets[1]);
    }

    // rest of buffers
    add_patch(pkt.get(), bps_cmd.handle, buf_desc[0].offset + offsetof(bps_tmp, settings_addr), bps_iq.handle, 0);
    add_patch(pkt.get(), bps_cmd.handle, buf_desc[0].offset + offsetof(bps_tmp, cdm_addr2), bps_cmd.handle, sizeof(bps_tmp));
    add_patch(pkt.get(), bps_cmd.handle, buf_desc[0].offset + 0xc8, bps_cdm_program_array.handle, 0);
    add_patch(pkt.get(), bps_cmd.handle, buf_desc[0].offset + offsetof(bps_tmp, striping_addr), bps_striping.handle, 0);
    add_patch(pkt.get(), bps_cmd.handle, buf_desc[0].offset + offsetof(bps_tmp, cdm_addr), bps_cdm_striping_bl.handle, 0);
  }

  int ret = device_config(m->icp_fd, session_handle, icp_dev_handle, cam_packet_handle);
  assert(ret == 0);
}

void SpectraCamera::config_ife(int idx, int request_id, bool init) {
  /*
    Handles initial + per-frame IFE config.
    * IFE = Image Front End
  */
  int size = sizeof(struct cam_packet) + sizeof(struct cam_cmd_buf_desc)*2;
  size += sizeof(struct cam_patch_desc)*10;
  if (!init) {
    size += sizeof(struct cam_buf_io_cfg);
  }

  uint32_t cam_packet_handle = 0;
  auto pkt = m->mem_mgr.alloc<struct cam_packet>(size, &cam_packet_handle);

  if (!init) {
    pkt->header.op_code =  CSLDeviceTypeIFE | OpcodesIFEUpdate;  // 0xf000001
    pkt->header.request_id = request_id;
  } else {
    pkt->header.op_code = CSLDeviceTypeIFE | OpcodesIFEInitialConfig; // 0xf000000
    pkt->header.request_id = 1;
  }
  pkt->header.size = size;

  // *** cmd buf ***
  std::vector<uint32_t> patches;
  {
    struct cam_cmd_buf_desc *buf_desc = (struct cam_cmd_buf_desc *)&pkt->payload;
    pkt->num_cmd_buf = 2;

    // *** first command ***
    buf_desc[0].size = ife_cmd.size;
    buf_desc[0].length = 0;
    buf_desc[0].type = CAM_CMD_BUF_DIRECT;
    buf_desc[0].meta_data = CAM_ISP_PACKET_META_COMMON;
    buf_desc[0].mem_handle = ife_cmd.handle;
    buf_desc[0].offset = ife_cmd.aligned_size()*idx;

    // stream of IFE register writes
    bool is_raw = cc.output_type != ISP_IFE_PROCESSED;
    if (!is_raw) {
      if (init) {
        buf_desc[0].length = build_initial_config((unsigned char*)ife_cmd.ptr + buf_desc[0].offset, cc, sensor.get(), patches, buf.out_img_width, buf.out_img_height);
      } else {
        const bool update_abf = ife_abf34_profile != ife_abf34_applied_profile;
        buf_desc[0].length = build_update((unsigned char*)ife_cmd.ptr + buf_desc[0].offset, cc, sensor.get(), patches,
                                          ife_abf34_profile, update_abf);
      }
    }

    pkt->kmd_cmd_buf_offset = buf_desc[0].length;
    pkt->kmd_cmd_buf_index = 0;

    // *** second command ***
    // parsed by cam_isp_packet_generic_blob_handler
    struct isp_packet {
      uint32_t type_0;
      cam_isp_resource_hfr_config resource_hfr;

      uint32_t type_1;
      cam_isp_clock_config clock;
      uint64_t extra_rdi_hz[3];

      uint32_t type_2;
      cam_isp_bw_config bw;
      struct cam_isp_bw_vote extra_rdi_vote[6];
    } __attribute__((packed)) tmp;
    memset(&tmp, 0, sizeof(tmp));

    tmp.type_0 = CAM_ISP_GENERIC_BLOB_TYPE_HFR_CONFIG;
    tmp.type_0 |= sizeof(cam_isp_resource_hfr_config) << 8;
    static_assert(sizeof(cam_isp_resource_hfr_config) == 0x20);
    tmp.resource_hfr = {
      .num_ports = 1,
      .port_hfr_config[0] = {
        .resource_type = static_cast<uint32_t>(is_raw ? CAM_ISP_IFE_OUT_RES_RDI_0 : CAM_ISP_IFE_OUT_RES_FULL),
        .subsample_pattern = 1,
        .subsample_period = 0,
        .framedrop_pattern = 1,
        .framedrop_period = 0,
      }
    };

    tmp.type_1 = CAM_ISP_GENERIC_BLOB_TYPE_CLOCK_CONFIG;
    tmp.type_1 |= (sizeof(cam_isp_clock_config) + sizeof(tmp.extra_rdi_hz)) << 8;
    static_assert((sizeof(cam_isp_clock_config) + sizeof(tmp.extra_rdi_hz)) == 0x38);
    tmp.clock = {
      .usage_type = 1, // dual mode
      .num_rdi = 4,
      .left_pix_hz = 404000000,
      .right_pix_hz = 404000000,
      .rdi_hz[0] = 404000000,
    };

    tmp.type_2 = CAM_ISP_GENERIC_BLOB_TYPE_BW_CONFIG;
    tmp.type_2 |= (sizeof(cam_isp_bw_config) + sizeof(tmp.extra_rdi_vote)) << 8;
    static_assert((sizeof(cam_isp_bw_config) + sizeof(tmp.extra_rdi_vote)) == 0xe0);
    tmp.bw = {
      .usage_type = 1, // dual mode
      .num_rdi = 4,
      .left_pix_vote = {
        .resource_id = 0,
        .cam_bw_bps = 450000000,
        .ext_bw_bps = 450000000,
      },
      .rdi_vote[0] = {
        .resource_id = 0,
        .cam_bw_bps = 8706200000,
        .ext_bw_bps = 8706200000,
      },
    };

    static_assert(offsetof(struct isp_packet, type_2) == 0x60);

    buf_desc[1].size = sizeof(tmp);
    buf_desc[1].offset = !init ? 0x60 : 0;
    buf_desc[1].length = buf_desc[1].size - buf_desc[1].offset;
    buf_desc[1].type = CAM_CMD_BUF_GENERIC;
    buf_desc[1].meta_data = CAM_ISP_PACKET_META_GENERIC_BLOB_COMMON;
    auto buf2 = m->mem_mgr.alloc<uint32_t>(buf_desc[1].size, (uint32_t*)&buf_desc[1].mem_handle);
    memcpy(buf2.get(), &tmp, sizeof(tmp));
  }

  // *** io config ***
  if (!init) {
    // configure output frame
    pkt->num_io_configs = 1;
    pkt->io_configs_offset = sizeof(struct cam_cmd_buf_desc)*pkt->num_cmd_buf;

    struct cam_buf_io_cfg *io_cfg = (struct cam_buf_io_cfg *)((char*)&pkt->payload + pkt->io_configs_offset);
    if (cc.output_type != ISP_IFE_PROCESSED) {
      io_cfg[0].mem_handle[0] = buf_handle_raw[idx];
      io_cfg[0].planes[0] = (struct cam_plane_cfg){
        .width = sensor->frame_width,
        .height = sensor->frame_height,
        .plane_stride = sensor->frame_stride,
        .slice_height = sensor->frame_height + sensor->extra_height,
      };
      io_cfg[0].format = sensor->mipi_format;
      io_cfg[0].color_space = CAM_COLOR_SPACE_BASE;
      io_cfg[0].color_pattern = 0x5;
      io_cfg[0].bpp = (sensor->mipi_format == CAM_FORMAT_MIPI_RAW_10 ? 0xa : 0xc);
      io_cfg[0].resource_type = CAM_ISP_IFE_OUT_RES_RDI_0;
      io_cfg[0].fence = sync_objs_ife[idx];
      io_cfg[0].direction = CAM_BUF_OUTPUT;
      io_cfg[0].subsample_pattern = 0x1;
      io_cfg[0].framedrop_pattern = 0x1;
    } else {
      io_cfg[0].mem_handle[0] = buf_handle_yuv[idx];
      io_cfg[0].mem_handle[1] = buf_handle_yuv[idx];
      io_cfg[0].planes[0] = (struct cam_plane_cfg){
        .width = buf.out_img_width,
        .height = buf.out_img_height,
        .plane_stride = stride,
        .slice_height = y_height,
      };
      io_cfg[0].planes[1] = (struct cam_plane_cfg){
        .width = buf.out_img_width,
        .height = buf.out_img_height / 2,
        .plane_stride = stride,
        .slice_height = uv_height,
      };
      io_cfg[0].offsets[1] = uv_offset;
      io_cfg[0].format = CAM_FORMAT_NV12;
      io_cfg[0].color_space = 0;
      io_cfg[0].color_pattern = 0x0;
      io_cfg[0].bpp = 0;
      io_cfg[0].resource_type = CAM_ISP_IFE_OUT_RES_FULL;
      io_cfg[0].fence = sync_objs_ife[idx];
      io_cfg[0].direction = CAM_BUF_OUTPUT;
      io_cfg[0].subsample_pattern = 0x1;
      io_cfg[0].framedrop_pattern = 0x1;
    }
  }

  // *** patches ***
  // sets up the kernel driver to do address translation for the IFE
  {
    // order here corresponds to the one in build_initial_config
    assert(patches.size() == 7 || patches.size() == 1 || patches.size() == 0);

    pkt->patch_offset = sizeof(struct cam_cmd_buf_desc)*pkt->num_cmd_buf + sizeof(struct cam_buf_io_cfg)*pkt->num_io_configs;
    if (patches.size() == 1) {
      const size_t n = sensor->abf34_noise_lut_profiles.size();
      const size_t p = std::min<size_t>(std::max(ife_abf34_profile, 0), n - 1);
      add_patch(pkt.get(), ife_cmd.handle, patches[0], ife_abf34_lut.handle,
                p * 64 * sizeof(uint32_t));
    } else if (patches.size() > 0) {
      // linearization LUT
      add_patch(pkt.get(), ife_cmd.handle, patches[0], ife_linearization_lut.handle, 0);

      // vignetting correction LUTs
      add_patch(pkt.get(), ife_cmd.handle, patches[1], ife_vignetting_lut.handle, 0);
      add_patch(pkt.get(), ife_cmd.handle, patches[2], ife_vignetting_lut.handle, ife_vignetting_lut.size);

      // gamma LUTs
      for (int i = 0; i < 3; i++) {
        add_patch(pkt.get(), ife_cmd.handle, patches[i+3], ife_gamma_lut.handle, ife_gamma_lut.size*i);
      }

      add_patch(pkt.get(), ife_cmd.handle, patches[6], ife_abf34_lut.handle, 0);
    }
  }

  int ret = device_config(m->isp_fd, session_handle, isp_dev_handle, cam_packet_handle);
  assert(ret == 0);
  if (!init) {
    ife_abf34_applied_profile = ife_abf34_profile;
  }
}

void SpectraCamera::enqueue_frame(uint64_t request_id) {
  int i = request_id % ife_buf_depth;
  assert(sync_objs_ife[i] == 0);

  // create output fences
  struct cam_sync_info sync_create = {0};
  strcpy(sync_create.name, "NodeOutputPortFence");
  int ret = do_sync_control(m->cam_sync_fd, CAM_SYNC_CREATE, &sync_create, sizeof(sync_create));
  if (ret != 0) {
    LOGE("failed to create fence: %d %d", ret, sync_create.sync_obj);
  } else {
    sync_objs_ife[i] = sync_create.sync_obj;
  }

  if (icp_dev_handle > 0) {
    ret = do_cam_control(m->cam_sync_fd, CAM_SYNC_CREATE, &sync_create, sizeof(sync_create));
    if (ret != 0) {
      LOGE("failed to create fence: %d %d", ret, sync_create.sync_obj);
    } else {
      sync_objs_bps[i] = sync_create.sync_obj;
    }
  }

  // schedule request with camera request manager
  struct cam_req_mgr_sched_request req_mgr_sched_request = {0};
  req_mgr_sched_request.session_hdl = session_handle;
  req_mgr_sched_request.link_hdl = link_handle;
  req_mgr_sched_request.req_id = request_id;
  ret = do_cam_control(m->video0_fd, CAM_REQ_MGR_SCHED_REQ, &req_mgr_sched_request, sizeof(req_mgr_sched_request));
  if (ret != 0) {
    LOGE("failed to schedule cam mgr request: %d %lu", ret, request_id);
  }

  // poke sensor, must happen after schedule
  sensors_poke(request_id);

  // submit request to IFE and BPS
  config_ife(i, request_id);
  if (cc.output_type == ISP_BPS_PROCESSED) config_bps(i, request_id);
}

void SpectraCamera::destroySyncObjectAt(int index) {
  auto destroy_sync_obj = [](int cam_sync_fd, int32_t &sync_obj) {
    if (sync_obj == 0) return;

    struct cam_sync_info sync_destroy = {.sync_obj = sync_obj};
    int ret = do_sync_control(cam_sync_fd, CAM_SYNC_DESTROY, &sync_destroy, sizeof(sync_destroy));
    if (ret != 0) {
      LOGE("Failed to destroy sync object: %d, sync_obj: %d", ret, sync_destroy.sync_obj);
    }

    sync_obj = 0;  // Reset the sync object to 0
  };

  destroy_sync_obj(m->cam_sync_fd, sync_objs_ife[index]);
  destroy_sync_obj(m->cam_sync_fd, sync_objs_bps[index]);
}

void SpectraCamera::camera_map_bufs() {
  int ret;
  for (int i = 0; i < ife_buf_depth; i++) {
    // map our VisionIPC bufs into ISP memory
    struct cam_mem_mgr_map_cmd mem_mgr_map_cmd = {0};
    mem_mgr_map_cmd.flags = CAM_MEM_FLAG_HW_READ_WRITE;
    mem_mgr_map_cmd.mmu_hdls[0] = m->device_iommu;
    mem_mgr_map_cmd.num_hdl = 1;
    if (icp_dev_handle > 0) {
      mem_mgr_map_cmd.num_hdl = 2;
      mem_mgr_map_cmd.mmu_hdls[1] = m->icp_device_iommu;
    }

    if (cc.output_type != ISP_IFE_PROCESSED) {
      // RAW bayer images
      mem_mgr_map_cmd.fd = buf.camera_bufs_raw[i].fd;
      ret = do_cam_control(m->video0_fd, CAM_REQ_MGR_MAP_BUF, &mem_mgr_map_cmd, sizeof(mem_mgr_map_cmd));
      assert(ret == 0);
      LOGD("map buf req: (fd: %d) 0x%x %d", buf.camera_bufs_raw[i].fd, mem_mgr_map_cmd.out.buf_handle, ret);
      buf_handle_raw[i] = mem_mgr_map_cmd.out.buf_handle;
    }

    if (cc.output_type != ISP_RAW_OUTPUT) {
      // final processed images
      VisionBuf *vb = buf.vipc_server->get_buffer(buf.stream_type, i);
      mem_mgr_map_cmd.fd = vb->fd;
      ret = do_cam_control(m->video0_fd, CAM_REQ_MGR_MAP_BUF, &mem_mgr_map_cmd, sizeof(mem_mgr_map_cmd));
      LOGD("map buf req: (fd: %d) 0x%x %d", vb->fd, mem_mgr_map_cmd.out.buf_handle, ret);
      buf_handle_yuv[i] = mem_mgr_map_cmd.out.buf_handle;
    }
  }
}

bool SpectraCamera::openSensor() {
  sensor_fd = open_v4l_by_name_and_index("cam-sensor-driver", cc.camera_num);
  assert(sensor_fd >= 0);
  LOGD("opened sensor for %d", cc.camera_num);

  LOGD("-- Probing sensor %d", cc.camera_num);

  auto init_sensor_lambda = [this](SensorInfo *s) {
    sensor.reset(s);
    return (sensors_init() == 0);
  };

  SensorInfo *candidate = cc.camera_num == 1
                            ? static_cast<SensorInfo *>(new S5K3M3)
                            : static_cast<SensorInfo *>(new IMX363);
  if (!init_sensor_lambda(candidate)) {
    LOGE("** sensor %d FAILED bringup, disabling", cc.camera_num);
    enabled = false;
    return false;
  }
  LOGD("-- Probing sensor %d success", cc.camera_num);

  // create session
  struct cam_req_mgr_session_info session_info = {};
  int ret = do_cam_control(m->video0_fd, CAM_REQ_MGR_CREATE_SESSION, &session_info, sizeof(session_info));
  LOGD("get session: %d 0x%X", ret, session_info.session_hdl);
  session_handle = session_info.session_hdl;

  // access the sensor
  LOGD("-- Accessing sensor");
  auto sensor_dev_handle_ = device_acquire(sensor_fd, session_handle, nullptr);
  assert(sensor_dev_handle_);
  sensor_dev_handle = *sensor_dev_handle_;
  LOGD("acquire sensor dev");

  LOG("-- Configuring sensor");
  if (cc.camera_num == 1 && sensor->init_reg_array.size() > 2) {
    // The S5K3M3 vendor table requires 3 ms after software reset (0x6010).
    sensors_i2c(sensor->init_reg_array.data(), 2, CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG, sensor->data_word);
    usleep(3000);
    sensors_i2c(sensor->init_reg_array.data() + 2, sensor->init_reg_array.size() - 2,
                CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG, sensor->data_word);
  } else {
    sensors_i2c(sensor->init_reg_array.data(), sensor->init_reg_array.size(),
                CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG, sensor->data_word);
  }

  // Both rear modules use movable lenses. Keep a separate calibration value for
  // each module because their actuators use different protocols and DAC ranges.
  const bool tele = cc.camera_num == 1;
  const char *focus_path = tele ? "/data/tele_focus_dac.txt" : "/data/focus_dac.txt";
  const unsigned focus_max = tele ? 1023 : 4095;
  // Tele DAC 220 is the measured far-focus peak for this Semco module.
  const unsigned focus_default = tele ? 220 : 1550;
  unsigned focus_dac = focus_default;
  if (FILE *f = fopen(focus_path, "r")) {
    const int parsed = fscanf(f, "%u", &focus_dac);
    fclose(f);
    if (parsed != 1 || focus_dac > focus_max) {
      LOGE("invalid %s; expected one DAC value in [0, %u]", focus_path, focus_max);
      focus_dac = focus_default;
    }
  }
  configActuator(static_cast<uint16_t>(focus_dac));

  if (tele) return true;

  // The Mi 8 rear Semco IMX363 module has a kernel OIS driver.  The stock
  // HAL configures it independently of the sensor request-manager link, so
  // keep it as a separate device here too.  This is optical stabilization;
  // it does not crop or warp the image as a future gyro EIS stage would.
  configOIS();
  return true;
}

bool SpectraCamera::actuator_i2c(const struct i2c_random_wr_payload *dat, int len, int op_code, bool data_word) {
  if (actuator_fd < 0 || actuator_dev_handle < 0) return false;

  uint32_t cam_packet_handle = 0;
  const int size = sizeof(struct cam_packet) + sizeof(struct cam_cmd_buf_desc);
  auto pkt = m->mem_mgr.alloc<struct cam_packet>(size, &cam_packet_handle);
  pkt->num_cmd_buf = 1;
  pkt->kmd_cmd_buf_index = -1;
  pkt->header.size = size;
  pkt->header.op_code = op_code;
  auto *buf_desc = reinterpret_cast<struct cam_cmd_buf_desc *>(&pkt->payload);

  buf_desc[0].size = buf_desc[0].length = sizeof(struct i2c_rdwr_header) + len * sizeof(struct i2c_random_wr_payload);
  buf_desc[0].type = CAM_CMD_BUF_I2C;
  auto i2c_random_wr = m->mem_mgr.alloc<struct cam_cmd_i2c_random_wr>(buf_desc[0].size, reinterpret_cast<uint32_t *>(&buf_desc[0].mem_handle));
  i2c_random_wr->header.count = len;
  i2c_random_wr->header.op_code = 1;
  i2c_random_wr->header.cmd_type = CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR;
  i2c_random_wr->header.data_type = data_word ? CAMERA_SENSOR_I2C_TYPE_WORD : CAMERA_SENSOR_I2C_TYPE_BYTE;
  i2c_random_wr->header.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
  memcpy(i2c_random_wr->random_wr_payload, dat, len * sizeof(*dat));

  const int ret = device_config(actuator_fd, session_handle, actuator_dev_handle, cam_packet_handle);
  if (ret != 0) LOGE("camera %d actuator focus move failed: %d", cc.camera_num, ret);
  return ret == 0;
}

void SpectraCamera::configActuator(uint16_t focus_dac) {
  const bool tele = cc.camera_num == 1;
  const char *actuator_name = tele ? "S5K3M3 AK7372" : "IMX363 AK7372";
  actuator_fd = open_v4l_by_name_and_index("cam-actuator-driver", cc.camera_num);
  if (actuator_fd < 0) {
    LOGE("%s actuator subdevice %d unavailable; fixed focus disabled", actuator_name, cc.camera_num);
    return;
  }

  auto dev_handle = device_acquire(actuator_fd, session_handle, nullptr);
  if (!dev_handle) {
    LOGE("failed to acquire %s actuator; fixed focus disabled", actuator_name);
    close(actuator_fd.fd_);
    actuator_fd.fd_ = -1;
    return;
  }
  actuator_dev_handle = *dev_handle;

  // This device has the Semco S5K3M3 variant. Its factory tuning uses an
  // AK7372 at 0x18 on CCI1, with a 10-bit code shifted into bits [15:6].
  const int num_cmd_buf = 2;
  uint32_t cam_packet_handle = 0;
  const int size = sizeof(struct cam_packet) + num_cmd_buf * sizeof(struct cam_cmd_buf_desc);
  auto pkt = m->mem_mgr.alloc<struct cam_packet>(size, &cam_packet_handle);
  pkt->num_cmd_buf = num_cmd_buf;
  pkt->kmd_cmd_buf_index = -1;
  pkt->header.size = size;
  pkt->header.op_code = CAM_ACTUATOR_PACKET_OPCODE_INIT;
  auto *buf_desc = reinterpret_cast<struct cam_cmd_buf_desc *>(&pkt->payload);

  buf_desc[0].size = buf_desc[0].length = sizeof(struct cam_cmd_i2c_info);
  buf_desc[0].type = CAM_CMD_BUF_LEGACY;
  auto i2c_info = m->mem_mgr.alloc<struct cam_cmd_i2c_info>(buf_desc[0].size, reinterpret_cast<uint32_t *>(&buf_desc[0].mem_handle));
  i2c_info->slave_addr = 0x18;
  i2c_info->i2c_freq_mode = I2C_FAST_MODE;
  i2c_info->cmd_type = CAMERA_SENSOR_CMD_TYPE_I2C_INFO;

  static const i2c_random_wr_payload kMainInit[] = {{0x02, 0x00}};
  static const i2c_random_wr_payload kTeleInit[] = {
    {0x00, 0x3c}, {0x01, 0x00}, {0x02, 0x00},
  };
  const i2c_random_wr_payload *init_regs = tele ? kTeleInit : kMainInit;
  const int init_count = tele ? sizeof(kTeleInit) / sizeof(kTeleInit[0])
                              : sizeof(kMainInit) / sizeof(kMainInit[0]);
  buf_desc[1].size = buf_desc[1].length = sizeof(struct i2c_rdwr_header) + init_count * sizeof(struct i2c_random_wr_payload);
  buf_desc[1].type = CAM_CMD_BUF_I2C;
  auto init_wr = m->mem_mgr.alloc<struct cam_cmd_i2c_random_wr>(buf_desc[1].size,
                 reinterpret_cast<uint32_t *>(&buf_desc[1].mem_handle));
  init_wr->header.count = init_count;
  init_wr->header.op_code = 1;
  init_wr->header.cmd_type = CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR;
  init_wr->header.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
  init_wr->header.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
  memcpy(init_wr->random_wr_payload, init_regs, init_count * sizeof(*init_regs));

  const int ret = device_config(actuator_fd, session_handle, actuator_dev_handle, cam_packet_handle);
  if (ret != 0) {
    LOGE("%s init failed: %d; fixed focus disabled", actuator_name, ret);
    device_control(actuator_fd, CAM_RELEASE_DEV, session_handle, actuator_dev_handle);
    actuator_dev_handle = -1;
    close(actuator_fd.fd_);
    actuator_fd.fd_ = -1;
    return;
  }

  if (tele) {
    const uint16_t focus_word = static_cast<uint16_t>(focus_dac << 6);
    const struct i2c_random_wr_payload focus_reg = {0x00, focus_word};
    actuator_i2c(&focus_reg, 1, CAM_ACTUATOR_PACKET_AUTO_MOVE_LENS, true);
    LOGD("S5K3M3 AK7372 tele fixed focus set to DAC %u", focus_dac);
    return;
  }

  // The factory actuator tuning uses a 12-bit position. AK737x stores that
  // position in bits [15:4] of the two-byte register at 0x00/0x01. The former
  // 10-bit split put the code in bits [9:0], so all non-zero focus requests
  // were scaled down by 16 and the focus scan barely moved the lens.
  const uint16_t focus_word = static_cast<uint16_t>(focus_dac << 4);
  const struct i2c_random_wr_payload focus_regs[] = {
    {0x00, static_cast<uint16_t>(focus_word >> 8)},
    {0x01, static_cast<uint16_t>(focus_word & 0xff)},
  };
  actuator_i2c(focus_regs, 2, CAM_ACTUATOR_PACKET_AUTO_MOVE_LENS, false);
  LOGD("AK7372 fixed focus set to DAC %u", focus_dac);
}

void SpectraCamera::configOIS() {
  ois_fd = open_v4l_by_name_and_index("cam-ois", 0);
  if (ois_fd < 0) {
    LOGE("IMX363 OIS subdevice unavailable; optical stabilization disabled");
    return;
  }

  auto dev_handle = device_acquire(ois_fd, session_handle, nullptr);
  if (!dev_handle) {
    LOGE("failed to acquire IMX363 OIS; optical stabilization disabled");
    close(ois_fd.fd_);
    ois_fd.fd_ = -1;
    return;
  }
  ois_dev_handle = *dev_handle;

  // Captured from Xiaomi's dipper camera HAL while starting the same Semco
  // IMX363 module.  The first command buffer is parsed as OIS slave/firmware
  // metadata, the second as init data and the third as factory calibration.
  // The downstream kernel then loads dipper_ois.prog and dipper_ois.coeff.
  static const i2c_random_wr_payload kInit[] = {
    {0x8262, 0xbf03}, {0x8263, 0x9f05}, {0x8264, 0x6040}, {0x8260, 0x1130},
    {0x8265, 0x8000}, {0x8261, 0x0280}, {0x8261, 0x0380}, {0x8261, 0x0988},
  };
  static const i2c_random_wr_payload kCalibration[] = {
    {0x847f, 0x0c0c}, {0x8436, 0xfd7f}, {0x8440, 0xff3f}, {0x8443, 0xf028},
    {0x841b, 0x8000}, {0x84b6, 0xfd7f}, {0x84c0, 0xff3f}, {0x84c3, 0xf028},
    {0x849b, 0x8000}, {0x8438, 0x3a14}, {0x84b8, 0x3a14}, {0x8447, 0x380d},
    {0x84c7, 0x380d}, {0x8290, 0xfd7f}, {0x8296, 0x8001}, {0x8291, 0x0500},
    {0x8292, 0x0200}, {0x8299, 0x8004}, {0x843a, 0x005a}, {0x84ba, 0x005a},
    {0x843b, 0x00c6}, {0x84bb, 0x00c6}, {0x847f, 0x0d0d},
  };

  uint32_t cam_packet_handle = 0;
  const int size = sizeof(cam_packet) + 3 * sizeof(cam_cmd_buf_desc);
  auto pkt = m->mem_mgr.alloc<cam_packet>(size, &cam_packet_handle);
  pkt->num_cmd_buf = 3;
  pkt->kmd_cmd_buf_index = -1;
  pkt->header.size = size;
  pkt->header.op_code = CAM_OIS_PACKET_OPCODE_INIT;
  auto *buf_desc = reinterpret_cast<cam_cmd_buf_desc *>(&pkt->payload);

  buf_desc[0].size = buf_desc[0].length = sizeof(cam_cmd_ois_info_v4l2);
  buf_desc[0].type = CAM_CMD_BUF_LEGACY;
  auto info = m->mem_mgr.alloc<cam_cmd_ois_info_v4l2>(buf_desc[0].size, reinterpret_cast<uint32_t *>(&buf_desc[0].mem_handle));
  info->slave_addr = 0x1c;  // 8-bit address; the CCI driver uses SID 0x0e.
  info->i2c_freq_mode = I2C_FAST_MODE;
  info->cmd_type = CAMERA_SENSOR_CMD_TYPE_I2C_INFO;
  info->ois_fw_flag = 1;
  info->is_ois_calib = 1;
  snprintf(info->ois_name, sizeof(info->ois_name), "%s", "dipper_ois");
  info->opcode = {0x80, 0x88, 0x82, 0x84};

  auto fill_i2c = [this, &buf_desc](int index, const i2c_random_wr_payload *regs, size_t count) {
    buf_desc[index].size = buf_desc[index].length = sizeof(i2c_rdwr_header) + count * sizeof(i2c_random_wr_payload);
    buf_desc[index].type = CAM_CMD_BUF_I2C;
    auto cmd = m->mem_mgr.alloc<cam_cmd_i2c_random_wr>(buf_desc[index].size, reinterpret_cast<uint32_t *>(&buf_desc[index].mem_handle));
    cmd->header.count = count;
    cmd->header.op_code = 1;
    cmd->header.cmd_type = CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR;
    cmd->header.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
    cmd->header.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
    memcpy(cmd->random_wr_payload, regs, count * sizeof(*regs));
  };
  fill_i2c(1, kInit, sizeof(kInit) / sizeof(kInit[0]));
  fill_i2c(2, kCalibration, sizeof(kCalibration) / sizeof(kCalibration[0]));

  int ret = device_config(ois_fd, session_handle, ois_dev_handle, cam_packet_handle);
  if (ret == 0) ret = device_control(ois_fd, CAM_START_DEV, session_handle, ois_dev_handle);
  if (ret == 0) {
    LOGD("IMX363 OIS initialized with stock Semco calibration");
    return;
  }

  LOGE("IMX363 OIS initialization failed: %d; continuing without OIS", ret);
  device_control(ois_fd, CAM_STOP_DEV, session_handle, ois_dev_handle);
  device_control(ois_fd, CAM_RELEASE_DEV, session_handle, ois_dev_handle);
  ois_dev_handle = -1;
  close(ois_fd.fd_);
  ois_fd.fd_ = -1;
}

void SpectraCamera::configISP() {
  if (!enabled) return;

  // ** ABI mismatch guard (xiaomi8/dipper) **
  // AGNOS's /usr/include/media/cam_isp.h has an extra `custom_csid` member that the Mi 8's
  // running kernel (stock downstream SDM845 4.9.200) does not have. Using the header struct
  // pushes `num_out_res` from offset 92 to 96, the kernel then reads our zeroed `reserved`,
  // sees num_out_res=0 and fails the acquire with "No PIX or RDI resource".
  // This mirrors the kernel's actual uapi layout (include/uapi/media/cam_isp.h, no custom_csid).
  struct cam_isp_in_port_info_dipper {
    uint32_t res_type, lane_type, lane_num, lane_cfg;
    uint32_t vc, dt, format;
    uint32_t test_pattern, usage_type;
    uint32_t left_start, left_stop, left_width;
    uint32_t right_start, right_stop, right_width;
    uint32_t line_start, line_stop, height;
    uint32_t pixel_clk, batch_size, dsp_mode, hbi_cnt;
    uint32_t reserved;
    uint32_t num_out_res;
    struct cam_isp_out_port_info data[1];
  };
  static_assert(sizeof(struct cam_isp_in_port_info_dipper) == 128,
                "cam_isp_in_port_info_dipper must match the running kernel's 128-byte layout");

  struct cam_isp_in_port_info_dipper in_port_info = {
    // ISP input to the CSID
    .res_type = cc.phy,
    .lane_type = CAM_ISP_LANE_TYPE_DPHY,
    .lane_num = 4,
    .lane_cfg = 0x3210,

    .vc = 0x0,
    .dt = sensor->frame_data_type,
    .format = sensor->mipi_format,

    .test_pattern = sensor->bayer_pattern,
    .usage_type = 0x0,

    .left_start = 0,
    .left_stop = sensor->frame_width - 1,
    .left_width = sensor->frame_width,

    .right_start = 0,
    .right_stop = sensor->frame_width - 1,
    .right_width = sensor->frame_width,

    .line_start = sensor->frame_offset,
    .line_stop = sensor->frame_height + sensor->frame_offset - 1,
    .height = sensor->frame_height + sensor->frame_offset,

    .pixel_clk = 0x0,
    .batch_size = 0x0,
    .dsp_mode = CAM_ISP_DSP_MODE_NONE,
    .hbi_cnt = 0x0,

    // ISP outputs
    .num_out_res = 0x1,
    .data[0] = (struct cam_isp_out_port_info){
      .res_type = CAM_ISP_IFE_OUT_RES_FULL,
      .format = CAM_FORMAT_NV12,
      .width = buf.out_img_width,
      .height = buf.out_img_height + sensor->extra_height,
      .comp_grp_id = 0x0, .split_point = 0x0, .secure_mode = 0x0,
    },
  };

  if (cc.output_type != ISP_IFE_PROCESSED) {
    in_port_info.line_start = 0;
    in_port_info.line_stop = sensor->frame_height + sensor->extra_height - 1;
    in_port_info.height = sensor->frame_height + sensor->extra_height;

    in_port_info.data[0].res_type = CAM_ISP_IFE_OUT_RES_RDI_0;
    in_port_info.data[0].format = sensor->mipi_format;
  }

  struct cam_isp_resource isp_resource = {
    .resource_id = CAM_ISP_RES_ID_PORT,
    .handle_type = CAM_HANDLE_USER_POINTER,
    .res_hdl = (uint64_t)&in_port_info,
    .length = sizeof(in_port_info),
  };

  auto isp_dev_handle_ = device_acquire(m->isp_fd, session_handle, &isp_resource);
  assert(isp_dev_handle_);
  isp_dev_handle = *isp_dev_handle_;
  LOGD("acquire isp dev");

  // allocate IFE memory, then configure it
  ife_cmd.init(m, 67984, 0x20, false, m->device_iommu, m->cdm_iommu, ife_buf_depth);
  if (cc.output_type == ISP_IFE_PROCESSED) {
    assert(sensor->gamma_lut_rgb.size() == 64);
    ife_gamma_lut.init(m, sensor->gamma_lut_rgb.size()*sizeof(uint32_t), 0x20, false, m->device_iommu, m->cdm_iommu, 3); // 3 for RGB
    for (int i = 0; i < 3; i++) {
      memcpy(ife_gamma_lut.ptr + ife_gamma_lut.size*i, sensor->gamma_lut_rgb.data(), ife_gamma_lut.size);
    }
    assert(sensor->linearization_lut.size() == 36);
    ife_linearization_lut.init(m, sensor->linearization_lut.size()*sizeof(uint32_t), 0x20, false, m->device_iommu, m->cdm_iommu);
    memcpy(ife_linearization_lut.ptr, sensor->linearization_lut.data(), ife_linearization_lut.size);
    assert(sensor->vignetting_lut.size() == 221);
    ife_vignetting_lut.init(m, sensor->vignetting_lut.size()*sizeof(uint32_t), 0x20, false, m->device_iommu, m->cdm_iommu, 2);
    // ife.h programs these as two *different* banks (DMI 14 = GRR, DMI 15 = GBB), so feeding
    // the GRR table to both applies the R/Gr shading gains to B/Gb. Use the sensor's own GBB
    // table when it has one; sensors that don't keep the previous duplicate-GRR behaviour.
    memcpy(ife_vignetting_lut.ptr, sensor->vignetting_lut.data(), ife_vignetting_lut.size);
    // DISABLE_GBB_LSC=1 forces the old duplicate-GRR behaviour, so the fix can be A/B tested
    // on one binary without disturbing the camera between captures.
    const bool has_gbb = sensor->vignetting_lut_gbb.size() == sensor->vignetting_lut.size() &&
                         getenv("DISABLE_GBB_LSC") == nullptr;
    memcpy(ife_vignetting_lut.ptr + ife_vignetting_lut.size,
           has_gbb ? sensor->vignetting_lut_gbb.data() : sensor->vignetting_lut.data(),
           ife_vignetting_lut.size);
    assert(sensor->abf34_noise_lut.size() == 64);
    const size_t abf_profiles = std::max<size_t>(1, sensor->abf34_noise_lut_profiles.size());
    ife_abf34_lut.init(m, sensor->abf34_noise_lut.size()*sizeof(uint32_t), 0x20, false,
                        m->device_iommu, m->cdm_iommu, abf_profiles);
    if (sensor->abf34_noise_lut_profiles.empty()) {
      memcpy(ife_abf34_lut.ptr, sensor->abf34_noise_lut.data(), ife_abf34_lut.size);
    } else {
      for (size_t i = 0; i < abf_profiles; ++i) {
        assert(sensor->abf34_noise_lut_profiles[i].size() == 64);
        memcpy(ife_abf34_lut.ptr + ife_abf34_lut.size*i,
               sensor->abf34_noise_lut_profiles[i].data(), ife_abf34_lut.size);
      }
    }
  }

  config_ife(0, 1, true);
}

void SpectraCamera::configICP() {
  /*
    Configures both the ICP and BPS.
  */

  int cfg_handle;

  uint32_t cfg_size = sizeof(bps_cfg[0]) / sizeof(bps_cfg[0][0]);
  void *cfg = alloc_w_mmu_hdl(m->video0_fd, cfg_size, (uint32_t*)&cfg_handle, 0x1,
                              CAM_MEM_FLAG_HW_READ_WRITE | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_HW_SHARED_ACCESS,
                              m->icp_device_iommu);
  memcpy(cfg, bps_cfg[sensor->num()], cfg_size);

  struct cam_icp_acquire_dev_info icp_info = {
    .scratch_mem_size = 0x0,
    .dev_type = CAM_ICP_RES_TYPE_BPS,
    .io_config_cmd_size = cfg_size,
    .io_config_cmd_handle = cfg_handle,
    .secure_mode = 0,
    .num_out_res = 1,
    .in_res = (struct cam_icp_res_info){
      .format = 0x9,  // RAW MIPI
      .width = sensor->frame_width,
      .height = sensor->frame_height,
      .fps = 20,
    },
    .out_res[0] = (struct cam_icp_res_info){
      .format = 0x3,  // YUV420NV12
      .width = buf.out_img_width,
      .height = buf.out_img_height,
      .fps = 20,
    },
  };
  auto h = device_acquire(m->icp_fd, session_handle, &icp_info);
  assert(h);
  icp_dev_handle = *h;
  LOGD("acquire icp dev");

  release(m->video0_fd, cfg_handle);

  // BPS has a lot of buffers to init
  bps_cmd.init(m, 464, 0x20, true, m->icp_device_iommu, 0, ife_buf_depth);

  // BPSIQSettings struct
  uint32_t settings_size = sizeof(bps_settings[0]) / sizeof(bps_settings[0][0]);
  bps_iq.init(m, settings_size, 0x20, true, m->icp_device_iommu);
  memcpy(bps_iq.ptr, bps_settings[sensor->num()], settings_size);

  // for cdm register writes, just make it bigger than you need
  bps_cdm_program_array.init(m, 0x1000, 0x20, true, m->icp_device_iommu);

  // striping lib output
  uint32_t striping_size = sizeof(bps_striping_output[0]) / sizeof(bps_striping_output[0][0]);
  bps_striping.init(m, striping_size, 0x20, true, m->icp_device_iommu);
  memcpy(bps_striping.ptr, bps_striping_output[sensor->num()], striping_size);

  // used internally by the BPS, we just allocate it.
  // size comes from the BPSStripingLib
  bps_cdm_striping_bl.init(m, 0xcfe0, 0x20, true, m->icp_device_iommu);

  if (sensor->out_scale > 1) {
    uint32_t full_stride, full_y_h, full_uv_h, full_yuv_size;
    std::tie(full_stride, full_y_h, full_uv_h, full_yuv_size) = get_nv12_info(sensor->frame_width, sensor->frame_height);
    bps_fullres_dummy.init(m, full_yuv_size, 0x1000, true, m->icp_device_iommu);
  }

  // LUTs
  assert(sensor->linearization_lut.size() == 36);
  bps_linearization_lut.init(m, sensor->linearization_lut.size()*sizeof(uint32_t), 0x20, true, m->icp_device_iommu);

  // bit shift linearization_lut to bps specs, also compensate for black level here
  uint32_t bl = sensor->black_level << (14 - sensor->bits_per_pixel);
  uint32_t* bps_lut = (uint32_t*)bps_linearization_lut.ptr;
  for (size_t i = 0; i < sensor->linearization_lut.size(); i++) {
    size_t seg = i / 4;
    size_t ch = i % 4;
    if (seg == 0) {
      bps_lut[i] = 0;
      continue;
    }
    uint32_t e = sensor->linearization_lut[(seg - 1) * 4 + ch];
    uint32_t base = e & 0x3fff;
    uint32_t slope_q11 = (e >> 14) & 0x3fff;
    uint32_t slope_q12 = std::min<uint32_t>(slope_q11 << 1, 0x3fff);
    base = (base > bl) ? (base - bl) : 0;
    bps_lut[i] = base | (slope_q12 << 14);
  }

  assert(sensor->gamma_lut_rgb.size() == 64);
  bps_gamma_lut.init(m, sensor->gamma_lut_rgb.size()*sizeof(uint32_t), 0x20, true, m->icp_device_iommu);
  memcpy(bps_gamma_lut.ptr, sensor->gamma_lut_rgb.data(), bps_gamma_lut.size);
}

void SpectraCamera::configCSIPHY() {
  csiphy_fd = open_v4l_by_name_and_index("cam-csiphy-driver", cc.camera_num);
  assert(csiphy_fd >= 0);
  LOGD("opened csiphy for %d", cc.camera_num);

  struct cam_csiphy_acquire_dev_info csiphy_acquire_dev_info = {.combo_mode = 0};
  auto csiphy_dev_handle_ = device_acquire(csiphy_fd, session_handle, &csiphy_acquire_dev_info);
  assert(csiphy_dev_handle_);
  csiphy_dev_handle = *csiphy_dev_handle_;
  LOGD("acquire csiphy dev");

  // config csiphy
  LOG("-- Config CSI PHY");
  {
    uint32_t cam_packet_handle = 0;
    int size = sizeof(struct cam_packet)+sizeof(struct cam_cmd_buf_desc)*1;
    auto pkt = m->mem_mgr.alloc<struct cam_packet>(size, &cam_packet_handle);
    pkt->num_cmd_buf = 1;
    pkt->kmd_cmd_buf_index = -1;
    pkt->header.size = size;
    struct cam_cmd_buf_desc *buf_desc = (struct cam_cmd_buf_desc *)&pkt->payload;

    buf_desc[0].size = buf_desc[0].length = sizeof(struct cam_csiphy_info);
    buf_desc[0].type = CAM_CMD_BUF_GENERIC;

    auto csiphy_info = m->mem_mgr.alloc<struct cam_csiphy_info>(buf_desc[0].size, (uint32_t*)&buf_desc[0].mem_handle);
    csiphy_info->lane_mask = 0x1f;
    csiphy_info->lane_assign = 0x3210;// skip clk. How is this 16 bit for 5 channels??
    csiphy_info->csiphy_3phase = 0x0; // no 3 phase, only 2 conductors per lane
    csiphy_info->combo_mode = 0x0;
    csiphy_info->lane_cnt = 0x4;
    csiphy_info->secure_mode = 0x0;
    csiphy_info->settle_time = MIPI_SETTLE_CNT * 200000000ULL;
    csiphy_info->data_rate = 48000000;  // Calculated by camera_freqs.py

    int ret_ = device_config(csiphy_fd, session_handle, csiphy_dev_handle, cam_packet_handle);
    assert(ret_ == 0);
  }
}

void SpectraCamera::linkDevices() {
  LOG("-- Link devices");
  struct cam_req_mgr_link_info req_mgr_link_info = {0};
  req_mgr_link_info.session_hdl = session_handle;
  req_mgr_link_info.num_devices = 2;
  req_mgr_link_info.dev_hdls[0] = isp_dev_handle;
  req_mgr_link_info.dev_hdls[1] = sensor_dev_handle;
  int ret = do_cam_control(m->video0_fd, CAM_REQ_MGR_LINK, &req_mgr_link_info, sizeof(req_mgr_link_info));
  assert(ret == 0);
  link_handle = req_mgr_link_info.link_hdl;
  LOGD("link: %d session: 0x%X isp: 0x%X sensors: 0x%X link: 0x%X", ret, session_handle, isp_dev_handle, sensor_dev_handle, link_handle);

  struct cam_req_mgr_link_control req_mgr_link_control = {0};
  req_mgr_link_control.ops = CAM_REQ_MGR_LINK_ACTIVATE;
  req_mgr_link_control.session_hdl = session_handle;
  req_mgr_link_control.num_links = 1;
  req_mgr_link_control.link_hdls[0] = link_handle;
  ret = do_cam_control(m->video0_fd, CAM_REQ_MGR_LINK_CONTROL, &req_mgr_link_control, sizeof(req_mgr_link_control));
  LOGD("link control: %d", ret);

  ret = device_control(csiphy_fd, CAM_START_DEV, session_handle, csiphy_dev_handle);
  LOGD("start csiphy: %d", ret);
  assert(ret == 0);
  ret = device_control(m->isp_fd, CAM_START_DEV, session_handle, isp_dev_handle);
  LOGD("start isp: %d", ret);
  assert(ret == 0);
  if (cc.output_type == ISP_BPS_PROCESSED) {
    ret = device_control(m->icp_fd, CAM_START_DEV, session_handle, icp_dev_handle);
    LOGD("start icp: %d", ret);
    assert(ret == 0);
  }
}

void SpectraCamera::camera_close() {
  LOG("-- Stop devices %d", cc.camera_num);

  if (enabled) {
    clear_req_queue();

    // ret = device_control(sensor_fd, CAM_STOP_DEV, session_handle, sensor_dev_handle);
    // LOGD("stop sensor: %d", ret);
    int ret = device_control(m->isp_fd, CAM_STOP_DEV, session_handle, isp_dev_handle);
    LOGD("stop isp: %d", ret);
    if (cc.output_type == ISP_BPS_PROCESSED) {
      ret = device_control(m->icp_fd, CAM_STOP_DEV, session_handle, icp_dev_handle);
      LOGD("stop icp: %d", ret);
    }
    ret = device_control(csiphy_fd, CAM_STOP_DEV, session_handle, csiphy_dev_handle);
    LOGD("stop csiphy: %d", ret);

    // link control stop
    LOG("-- Stop link control");
    struct cam_req_mgr_link_control req_mgr_link_control = {0};
    req_mgr_link_control.ops = CAM_REQ_MGR_LINK_DEACTIVATE;
    req_mgr_link_control.session_hdl = session_handle;
    req_mgr_link_control.num_links = 1;
    req_mgr_link_control.link_hdls[0] = link_handle;
    ret = do_cam_control(m->video0_fd, CAM_REQ_MGR_LINK_CONTROL, &req_mgr_link_control, sizeof(req_mgr_link_control));
    LOGD("link control stop: %d", ret);

    // unlink
    LOG("-- Unlink");
    struct cam_req_mgr_unlink_info req_mgr_unlink_info = {0};
    req_mgr_unlink_info.session_hdl = session_handle;
    req_mgr_unlink_info.link_hdl = link_handle;
    ret = do_cam_control(m->video0_fd, CAM_REQ_MGR_UNLINK, &req_mgr_unlink_info, sizeof(req_mgr_unlink_info));
    LOGD("unlink: %d", ret);

    // release devices
    LOGD("-- Release devices");
    ret = device_control(m->isp_fd, CAM_RELEASE_DEV, session_handle, isp_dev_handle);
    LOGD("release isp: %d", ret);
    if (cc.output_type == ISP_BPS_PROCESSED) {
      ret = device_control(m->icp_fd, CAM_RELEASE_DEV, session_handle, icp_dev_handle);
      LOGD("release icp: %d", ret);
    }
    ret = device_control(csiphy_fd, CAM_RELEASE_DEV, session_handle, csiphy_dev_handle);
    LOGD("release csiphy: %d", ret);

    for (int i = 0; i < ife_buf_depth; i++) {
      if (buf_handle_raw[i]) {
        release(m->video0_fd, buf_handle_raw[i]);
      }
      if (buf_handle_yuv[i]) {
        release(m->video0_fd, buf_handle_yuv[i]);
      }
    }
    LOGD("released buffers");
  }

  int ret = device_control(sensor_fd, CAM_RELEASE_DEV, session_handle, sensor_dev_handle);
  LOGD("release sensor: %d", ret);

  if (actuator_dev_handle >= 0) {
    ret = device_control(actuator_fd, CAM_RELEASE_DEV, session_handle, actuator_dev_handle);
    LOGD("release AK7372 actuator: %d", ret);
    actuator_dev_handle = -1;
  }

  // The downstream OIS driver rejects RELEASE while it is in START state.
  // Keep this order even when the rest of the camera pipeline has already
  // stopped so a later camerad restart can power the OIS cleanly.
  if (ois_dev_handle >= 0) {
    ret = device_control(ois_fd, CAM_STOP_DEV, session_handle, ois_dev_handle);
    LOGD("stop IMX363 OIS: %d", ret);
    ret = device_control(ois_fd, CAM_RELEASE_DEV, session_handle, ois_dev_handle);
    LOGD("release IMX363 OIS: %d", ret);
    ois_dev_handle = -1;
  }

  // destroyed session
  struct cam_req_mgr_session_info session_info = {.session_hdl = session_handle};
  ret = do_cam_control(m->video0_fd, CAM_REQ_MGR_DESTROY_SESSION, &session_info, sizeof(session_info));
  LOGD("destroyed session %d: %d", cc.camera_num, ret);
}

bool SpectraCamera::handle_camera_event(const cam_req_mgr_message *event_data) {
  /*
    Handles camera SOF event. Returns true if the frame is valid for publishing.
  */

  uint64_t request_id = event_data->u.frame_msg.request_id;  // ID from the camera request manager
  uint64_t frame_id_raw = event_data->u.frame_msg.frame_id;  // raw as opposed to our re-indexed frame ID
  uint64_t timestamp = event_data->u.frame_msg.timestamp;    // timestamped in the kernel's SOF IRQ callback
  //LOGD("handle cam %d ts %lu req id %lu frame id %lu", cc.camera_num, timestamp, request_id, frame_id_raw);

  // if there's a lag, some more frames could have already come in before
  // we cleared the queue, so we'll still get them with valid (> 0) request IDs.
  if (timestamp < last_requeue_ts) {
    LOGD("skipping frame: ts before requeue / cam %d ts %lu req id %lu frame id %lu", cc.camera_num, timestamp, request_id, frame_id_raw);
    return false;
  }

  if (stress_test("skipping SOF event")) {
    return false;
  }

  if (!validateEvent(request_id, frame_id_raw)) {
    return false;
  }

  // Update tracking variables
  if (request_id == request_id_last + 1) {
    skip_expected = false;
  }
  frame_id_raw_last = frame_id_raw;
  request_id_last = request_id;

  // Wait until frame's fully read out and processed
  if (!waitForFrameReady(request_id)) {
    // Reset queue on sync failure to prevent frame tearing
    LOGE("camera %d sync failure %ld %ld ", cc.camera_num, request_id, frame_id_raw);
    clearAndRequeue(request_id + 1);
    return false;
  }

  int buf_idx = request_id % ife_buf_depth;
  bool ret = processFrame(buf_idx, request_id, frame_id_raw, timestamp);
  destroySyncObjectAt(buf_idx);
  enqueue_frame(request_id + ife_buf_depth);  // request next frame for this slot
  return ret;
}

bool SpectraCamera::validateEvent(uint64_t request_id, uint64_t frame_id_raw) {
  // check if the request ID is even valid. this happens after queued
  // requests are cleared. unclear if it happens any other time.
  if (request_id == 0) {
    if (invalid_request_count++ > ife_buf_depth+2) {
      LOGE("camera %d reset after half second of invalid requests", cc.camera_num);
      clearAndRequeue(request_id_last + 1);
      invalid_request_count = 0;
    }
    return false;
  }
  invalid_request_count = 0;

  // check for skips in frame_id or request_id
  if (!skip_expected) {
    if (frame_id_raw != frame_id_raw_last + 1) {
      LOGE("camera %d frame ID skipped, %lu -> %lu", cc.camera_num, frame_id_raw_last, frame_id_raw);
      clearAndRequeue(request_id + 1);
      return false;
    }

    if (request_id != request_id_last + 1) {
      LOGE("camera %d requests skipped %ld -> %ld", cc.camera_num, request_id_last, request_id);
      clearAndRequeue(request_id + 1);
      return false;
    }
  }
  return true;
}

void SpectraCamera::clearAndRequeue(uint64_t from_request_id) {
  // clear everything, then queue up a fresh set of frames
  LOGW("clearing and requeuing camera %d from %lu", cc.camera_num, from_request_id);
  clear_req_queue();
  last_requeue_ts = nanos_since_boot();
  for (uint64_t id = from_request_id; id < from_request_id + ife_buf_depth; ++id) {
    enqueue_frame(id);
  }
  skip_expected = true;
}

bool SpectraCamera::waitForFrameReady(uint64_t request_id) {
  int buf_idx = request_id % ife_buf_depth;
  assert(sync_objs_ife[buf_idx]);

  if (stress_test("sync sleep time")) {
    util::sleep_for(350);
    return false;
  }

  auto waitForSync = [&](uint32_t sync_obj, int timeout_ms, const char *sync_type) {
    double st = millis_since_boot();
    struct cam_sync_wait sync_wait = {};
    sync_wait.sync_obj = sync_obj;
    sync_wait.timeout_ms = stress_test(sync_type) ? 1 : timeout_ms;
    bool ret = do_sync_control(m->cam_sync_fd, CAM_SYNC_WAIT, &sync_wait, sizeof(sync_wait)) == 0;
    double et = millis_since_boot();
    if (!ret) LOGE("camera %d %s failed after %.2fms", cc.camera_num, sync_type, et-st);
    return ret;
  };

  // wait for frame from IFE
  // - in RAW_OUTPUT mode, this time is just the frame readout from the sensor
  // - in IFE_PROCESSED mode, this time also includes image processing (~1ms)
  bool success = waitForSync(sync_objs_ife[buf_idx], 100, "IFE sync");
  if (success && sync_objs_bps[buf_idx]) {
    // BPS is typically 7ms
    success = waitForSync(sync_objs_bps[buf_idx], 50, "BPS sync");
  }

  return success;
}

bool SpectraCamera::processFrame(int buf_idx, uint64_t request_id, uint64_t frame_id_raw, uint64_t timestamp) {
  // Unlike comma 3/3X, the two Mi 8 rear sensors have no shared frame-sync
  // trigger. Their independent PLLs differ by a few microseconds per frame,
  // which grows beyond modeld's synchronization window. Once per second,
  // trim the tele sensor's frame length around its nominal value. This runs on
  // a different phase than AE, so it does not make the downstream kernel issue
  // simultaneous diagnostic I2C readbacks for both operations.
  static std::array<std::atomic<uint64_t>, 2> mi8_sof = {};
  static std::array<std::atomic<uint64_t>, 2> mi8_raw_id = {};
  static int mi8_tele_fll = 3428;
  static std::array<int64_t, 9> mi8_phase_error_window = {};
  static size_t mi8_phase_error_count = 0;
  static size_t mi8_phase_error_index = 0;
  const bool mi8_dual = getenv("ENABLE_TELE") != nullptr ||
                        access("/data/enable_mi8_tele_camera", F_OK) == 0;
  if (mi8_dual && cc.camera_num <= 1) {
    mi8_sof[cc.camera_num].store(timestamp, std::memory_order_relaxed);
    mi8_raw_id[cc.camera_num].store(frame_id_raw, std::memory_order_relaxed);
    const uint64_t main_sof = mi8_sof[0].load(std::memory_order_relaxed);
    if (cc.camera_num == 1 && main_sof != 0) {
      constexpr int64_t frame_period_ns = 50000000;
      constexpr int64_t half_period_ns = frame_period_ns / 2;
      constexpr int64_t target_phase_ns = 100000;  // tele follows main by 0.1 ms
      constexpr int64_t tele_line_ns = 14580;
      constexpr int correction_frames = 17;  // allow for sensor register latency
      constexpr int nominal_fll = 3428;

      int64_t phase_ns = static_cast<int64_t>(timestamp) - static_cast<int64_t>(main_sof);
      while (phase_ns > half_period_ns) phase_ns -= frame_period_ns;
      while (phase_ns < -half_period_ns) phase_ns += frame_period_ns;

      const int64_t phase_error_ns = phase_ns - target_phase_ns;
      mi8_phase_error_window[mi8_phase_error_index] = phase_error_ns;
      mi8_phase_error_index = (mi8_phase_error_index + 1) % mi8_phase_error_window.size();
      mi8_phase_error_count = std::min(mi8_phase_error_count + 1, mi8_phase_error_window.size());

      if (frame_id_raw % 20 == 2) {
        // The Mi 8 kernel timestamps SOF in its IRQ callback. Under sustained
        // GPU load that callback can be delayed by several milliseconds even
        // though the exposure itself stayed in phase. Filter the last nine
        // circular phase samples before changing sensor timing so a scheduling
        // spike cannot make the controller pull a good sensor out of lock.
        auto sorted_errors = mi8_phase_error_window;
        std::sort(sorted_errors.begin(), sorted_errors.begin() + mi8_phase_error_count);
        const int64_t filtered_phase_error_ns = sorted_errors[mi8_phase_error_count / 2];
        constexpr int64_t large_error_ns = 1500000;
        const bool startup_lock = !first_frame_synced;
        const bool large_error = std::abs(filtered_phase_error_ns) > large_error_ns;
        const double correction = static_cast<double>(-filtered_phase_error_ns) /
                                  static_cast<double>(tele_line_ns * correction_frames);
        const int correction_limit = (startup_lock || large_error) ? 20 : 4;
        const int correction_lines = std::clamp(static_cast<int>(std::llround(correction)),
                                                -correction_limit, correction_limit);
        const int requested_fll = std::clamp(nominal_fll + correction_lines, 3408, 3448);
        if (requested_fll != mi8_tele_fll) {
          const i2c_random_wr_payload fll_reg = {0x0340, static_cast<uint16_t>(requested_fll)};
          sensors_i2c(&fll_reg, 1, CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG, sensor->data_word);
          mi8_tele_fll = requested_fll;
          if (startup_lock || large_error || std::abs(correction_lines) > 2) {
            LOGW("Mi 8 tele raw phase %.3f ms, median error %.3f ms, setting FLL %d",
                 phase_ns / 1e6, filtered_phase_error_ns / 1e6, requested_fll);
          }
        }
      }
    }
  }

  if (!syncFirstFrame(cc.camera_num, request_id, frame_id_raw, timestamp, false)) {  // 2026-08-30: single camera, no staggered SOF
    return false;
  }

  if (mi8_dual && cc.camera_num <= 1) {
    // A kernel queue recovery can jump one camera's raw counter without
    // restarting camerad. Keep camera 0 as the sequence authority and realign
    // the tele offset only when both latest SOFs are clearly the same physical
    // exposure. Run this check after either camera event: depending on their
    // sub-millisecond phase, tele can arrive just before main, in which case a
    // tele-only check compares against main's previous (50 ms old) exposure
    // and can never recover a one-frame startup offset. This preserves shared
    // IDs across requeues without rounding SOF timestamps to a fixed clock grid.
    const uint64_t main_sof = mi8_sof[0].load(std::memory_order_relaxed);
    const uint64_t tele_sof = mi8_sof[1].load(std::memory_order_relaxed);
    const uint64_t sof_delta = (tele_sof > main_sof) ? tele_sof - main_sof : main_sof - tele_sof;
    constexpr uint64_t same_exposure_tolerance_ns = 10000000;
    if (main_sof != 0 && tele_sof != 0 && sof_delta <= same_exposure_tolerance_ns) {
      const uint64_t main_raw_id = mi8_raw_id[0].load(std::memory_order_relaxed);
      const uint64_t tele_raw_id = mi8_raw_id[1].load(std::memory_order_relaxed);
      const uint64_t main_frame_id = main_raw_id - camera_sync_data[0].frame_id_offset;
      const uint64_t tele_frame_id = tele_raw_id - camera_sync_data[1].frame_id_offset;
      if (tele_frame_id != main_frame_id && tele_raw_id >= main_frame_id) {
        LOGW("Mi 8 realigning tele frame ID %lu -> %lu (SOF delta %.3f ms)",
             tele_frame_id, main_frame_id, sof_delta / 1e6);
        camera_sync_data[1].frame_id_offset = tele_raw_id - main_frame_id;
      }
    }
  }

  // in IFE_PROCESSED mode, we can't know the true EOF, so recover it with sensor readout time
  uint64_t timestamp_eof = timestamp + sensor->readout_time_ns;

  // Update buffer and frame data
  buf.cur_buf_idx = buf_idx;
  // Keep the public sequence tied to each sensor's monotonic hardware counter.
  // syncFirstFrame() records an offset only after both SOFs are aligned, so the
  // two counters describe the same physical exposure. Deriving IDs by rounding
  // timestamps to an ideal 50 ms grid is unsafe: the real oscillators drift
  // relative to that grid and two genuinely synchronized SOFs can temporarily
  // land on opposite sides of a rounding boundary.
  uint32_t frame_id = (uint32_t)(frame_id_raw - camera_sync_data[cc.camera_num].frame_id_offset);
  buf.cur_frame_data = {
    .frame_id = frame_id,
    .request_id = (uint32_t)request_id,
    .timestamp_sof = timestamp,
    .timestamp_eof = timestamp_eof,
    .processing_time = float((nanos_since_boot() - timestamp_eof) * 1e-9)
  };
  return true;
}

bool SpectraCamera::syncFirstFrame(int camera_id, uint64_t request_id, uint64_t raw_id, uint64_t timestamp, bool staggered) {
  if (first_frame_synced) return true;

  // Store the frame data for this camera
  camera_sync_data[camera_id] = SyncData{timestamp, raw_id + 1, staggered};

  // Ensure all cameras are up
  int enabled_camera_count = std::count_if(std::begin(ALL_CAMERA_CONFIGS), std::end(ALL_CAMERA_CONFIGS),
                                           [](const auto &config) { return config.enabled; });
  bool all_cams_up = camera_sync_data.size() == enabled_camera_count;

  // Check that camera timestamps are properly aligned:
  // - non-staggered cameras should be within 0.2ms of each other
  // - staggered cameras should be within 0.2ms of a 25ms offset from non-staggered cameras
  const uint64_t half_period_ns = 25 * 1000000ULL;  // 25ms
  const uint64_t tolerance_ns = 200000ULL;           // 0.2ms
  bool all_cams_synced = true;
  for (const auto &[cam, sync_data] : camera_sync_data) {
    if (cam == camera_id) continue;
    uint64_t diff = std::max(timestamp, sync_data.timestamp) -
                    std::min(timestamp, sync_data.timestamp);
    bool pair_staggered = staggered != sync_data.staggered;
    uint64_t expected_offset = pair_staggered ? half_period_ns : 0;
    uint64_t error = (diff > expected_offset) ? diff - expected_offset : expected_offset - diff;
    if (error > tolerance_ns) {
      all_cams_synced = false;
    }
  }

  if (all_cams_up && all_cams_synced) {
    first_frame_synced = true;
    sync_epoch_timestamp = timestamp;
    for (const auto &[cam, sync_data] : camera_sync_data) {
      sync_epoch_timestamp = std::min(sync_epoch_timestamp, sync_data.timestamp);
    }
    for (const auto&[cam, sync_data] : camera_sync_data) {
      LOGW("camera %d synced on frame_id_offset %ld timestamp %lu", cam, sync_data.frame_id_offset, sync_data.timestamp);
    }
  }

  // Timeout in case the timestamps never line up
  if (raw_id > 80) {
    LOGE("camera first frame sync timed out");
    first_frame_synced = true;
  }

  return false;
}
