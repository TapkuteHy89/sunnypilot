#!/usr/bin/env python3
"""Per-sensor adaptive white balance for the Mi 8 dual-camera IFE path.

The V4L2 IFE path produces NV12 directly and therefore bypasses camerad's
RAW Bayer Grey-World AWB. This low-rate controller measures near-neutral pixels
from each VisionIPC stream and updates a separate IFE gain file for each sensor.
"""

from __future__ import annotations

import json
import math
import os
import time
from functools import lru_cache
from pathlib import Path

import numpy as np

from msgq.visionipc import VisionIpcClient
from openpilot.cereal.visionipc import VisionStreamType
from openpilot.common.swaglog import cloudlog


STATUS_FILE = Path("/tmp/mi8-camera-awb-status.json")
UPDATE_INTERVAL_SECONDS = 3.0
MIN_GAIN = 0.65
ERROR_DEADBAND = 0.025
MIN_NEUTRAL_LUMA = 20.0
MIN_FALLBACK_LUMA = 15.0

SOURCES = (
  {
    "name": "imx363_wide",
    "source_stream": "wideRoadCameraState/IMX363",
    "stream_type": VisionStreamType.VISION_STREAM_WIDE_ROAD,
    "gain_file": Path("/data/wb_gains.txt"),
    "fallback_gains": (1.0, 1.6610, 1.7400),
    # Live neutral-pixel feedback reaches 1.80 on both red and blue while
    # retaining a repeatable 4-6% residual error. Give the controller enough
    # calibrated headroom to converge instead of baking a green cast into the
    # IMX363 stream. This remains well below the tele sensor's 3.0 guardrail.
    "max_gain": 2.20,
  },
  {
    "name": "s5k3m3_tele",
    "source_stream": "narrowRoadCameraState/S5K3M3",
    "stream_type": VisionStreamType.VISION_STREAM_NARROW_ROAD,
    "gain_file": Path("/data/tele_wb_gains.txt"),
    "fallback_gains": (1.0, 1.4370, 1.9473),
    "max_gain": 3.00,
  },
)


def read_gains(source: dict) -> tuple[float, float, float]:
  try:
    values = tuple(float(item) for item in source["gain_file"].read_text().split())
    if len(values) == 3 and all(MIN_GAIN <= value <= source["max_gain"] for value in values):
      return values
  except (OSError, ValueError):
    pass
  return source["fallback_gains"]


def write_gains(source: dict, gains: tuple[float, float, float]) -> None:
  gain_file = source["gain_file"]
  temporary = gain_file.with_suffix(".awb-tmp")
  temporary.write_text("%.4f %.4f %.4f\n" % gains)
  temporary.chmod(0o644)
  os.replace(temporary, gain_file)


@lru_cache(maxsize=4)
def sample_indices(width: int, height: int, stride: int, uv_offset: int) -> tuple[np.ndarray, np.ndarray]:
  yy, xx = np.meshgrid(
    np.arange(height // 10, 9 * height // 10, 8, dtype=np.int64),
    np.arange(width // 10, 9 * width // 10, 8, dtype=np.int64),
    indexing="ij",
  )
  yy = yy.ravel() & ~1
  xx = xx.ravel() & ~1
  return yy * stride + xx, uv_offset + (yy // 2) * stride + xx


def neutral_mean(frame) -> tuple[np.ndarray, int, float, float]:
  y_index, uv_index = sample_indices(frame.width, frame.height, frame.stride, frame.uv_offset)
  # Current msgq exposes VisionBuf.data as a memoryview. Convert it to a
  # zero-copy ndarray before using vector indices; indexing a memoryview with
  # a NumPy array raises TypeError on the current runtime.
  data = np.frombuffer(frame.data, dtype=np.uint8)
  y = data[y_index].astype(np.float32)
  u = data[uv_index].astype(np.float32) - 128.0
  v = data[uv_index + 1].astype(np.float32) - 128.0

  # Use the exact matrix from system.camerad.snapshot.yuv_to_rgb.  The previous
  # full-range BT.601 coefficients did not match the RGB image consumed and
  # displayed by openpilot, so the controller could report neutral RGB while
  # the actual wide-camera image still had a green cast.
  pixels = np.stack((
    np.clip(y + 1.13983 * v, 0.0, 255.0),
    np.clip(y - 0.39465 * u - 0.58060 * v, 0.0, 255.0),
    np.clip(y + 2.03211 * u, 0.0, 255.0),
  ), axis=1)
  luma = pixels.mean(axis=1)
  saturation = pixels.max(axis=1) - pixels.min(axis=1)

  # Near-black pixels are sensor noise, not a white reference.  At night the
  # old 5-code floor accepted almost every pixel (99.9% in the failing frame)
  # and steered WB from the colour of the black level.  Prefer the brighter
  # portion of the scene while retaining a fixed floor for dark roads.
  luma_floor = max(MIN_NEUTRAL_LUMA, float(np.percentile(luma, 65.0)))
  mask = (luma >= luma_floor) & (luma <= 240.0) & (saturation <= 20.0)
  if int(mask.sum()) < 500:
    luma_floor = MIN_FALLBACK_LUMA
    mask = (luma >= luma_floor) & (luma <= 240.0) & (saturation <= 25.0)
  count = int(mask.sum())
  if count < 200:
    raise RuntimeError(f"too few near-neutral pixels: {count}")
  return pixels[mask].mean(axis=0), count, count / len(pixels), luma_floor


def save_status(payload: dict) -> None:
  temporary = STATUS_FILE.with_suffix(".tmp")
  temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
  os.replace(temporary, STATUS_FILE)


def run_controller(clients: list[tuple[dict, VisionIpcClient]]) -> None:
  states = {}
  for source, _ in clients:
    gain_g, gain_b, gain_r = read_gains(source)
    states[source["name"]] = {
      "gain_g": gain_g,
      "gain_b": gain_b,
      "gain_r": gain_r,
      "filtered_error_r": 0.0,
      "filtered_error_b": 0.0,
      "update_count": 0,
    }

  while True:
    time.sleep(UPDATE_INTERVAL_SECONDS)
    stream_status = {}
    for source, client in clients:
      state = states[source["name"]]
      frame = client.recv()
      if frame is None:
        raise RuntimeError(f"{source['source_stream']} ended")

      mean, sample_count, sample_fraction, luma_floor = neutral_mean(frame)
      red, green, blue = (float(value) for value in mean)
      error_r = math.log(max(green, 1.0) / max(red, 1.0))
      error_b = math.log(max(green, 1.0) / max(blue, 1.0))
      state["filtered_error_r"] = 0.55 * state["filtered_error_r"] + 0.45 * error_r
      state["filtered_error_b"] = 0.55 * state["filtered_error_b"] + 0.45 * error_b

      controlled_r = 0.0 if abs(state["filtered_error_r"]) < ERROR_DEADBAND else state["filtered_error_r"]
      controlled_b = 0.0 if abs(state["filtered_error_b"]) < ERROR_DEADBAND else state["filtered_error_b"]
      max_log_step = math.log(1.08 if state["update_count"] < 12 else 1.04)
      step_r = math.exp(float(np.clip(0.35 * controlled_r, -max_log_step, max_log_step)))
      step_b = math.exp(float(np.clip(0.35 * controlled_b, -max_log_step, max_log_step)))
      next_r = float(np.clip(state["gain_r"] * step_r, MIN_GAIN, source["max_gain"]))
      next_b = float(np.clip(state["gain_b"] * step_b, MIN_GAIN, source["max_gain"]))
      changed = abs(next_r - state["gain_r"]) >= 0.002 or abs(next_b - state["gain_b"]) >= 0.002
      if changed:
        state["gain_r"], state["gain_b"] = next_r, next_b
        write_gains(source, (state["gain_g"], state["gain_b"], state["gain_r"]))

      source_status = {
        "source_stream": source["source_stream"],
        "gain_file": str(source["gain_file"]),
        "updates": state["update_count"],
        "gain_file_order": "G B R",
        "gains_g_b_r": [round(state["gain_g"], 4), round(state["gain_b"], 4), round(state["gain_r"], 4)],
        "neutral_mean_rgb": [round(red, 3), round(green, 3), round(blue, 3)],
        "neutral_samples": sample_count,
        "neutral_fraction": round(sample_fraction, 4),
        "neutral_luma_floor": round(luma_floor, 3),
        "filtered_log_error_r": round(state["filtered_error_r"], 5),
        "filtered_log_error_b": round(state["filtered_error_b"], 5),
        "wrote_gain_file": changed,
      }
      stream_status[source["name"]] = source_status
      if state["update_count"] < 12 or state["update_count"] % 20 == 0:
        cloudlog.info(f"camera_awbd {source['name']} {source_status}")
      state["update_count"] += 1

    save_status({
      "active": True,
      "mode": "per-sensor-dual-stream",
      "timestamp_monotonic": time.monotonic(),
      "streams": stream_status,
    })


def main() -> None:
  cloudlog.info("Mi 8 per-sensor adaptive camera white balance starting")
  while True:
    try:
      clients = []
      for source in SOURCES:
        client = VisionIpcClient("camerad", source["stream_type"], True)
        if not client.connect(True):
          raise RuntimeError(f"VisionIPC connection failed for {source['source_stream']}")
        clients.append((source, client))
      run_controller(clients)
    except KeyboardInterrupt:
      return
    except Exception:
      cloudlog.exception("camera_awbd restarting after error")
      time.sleep(2.0)


if __name__ == "__main__":
  main()
