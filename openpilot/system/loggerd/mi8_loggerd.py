"""Recover interrupted Mi 8 log tails before starting the native logger.

Only the last two segments of the newest route can have been in flight during
a rotation. Preserve originals outside realdata before trimming any suffix.
"""
import ctypes
import io
import json
import mmap
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time
import uuid

import zstandard

SEGMENT = re.compile(r"^([0-9a-f]{8}--[0-9a-f]{10})--([0-9]+)$")


def sync_dir(path: Path) -> None:
  fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
  try:
    os.fsync(fd)
  finally:
    os.close(fd)


def valid_prefix(path: Path) -> int:
  if path.stat().st_size == 0:
    return 0
  lib = ctypes.CDLL("libzstd.so.1")
  lib.ZSTD_findFrameCompressedSize.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
  lib.ZSTD_findFrameCompressedSize.restype = ctypes.c_size_t
  lib.ZSTD_isError.argtypes = [ctypes.c_size_t]
  lib.ZSTD_isError.restype = ctypes.c_uint
  pos = 0
  with path.open("rb") as source, mmap.mmap(source.fileno(), 0, access=mmap.ACCESS_COPY) as data:
    address = ctypes.addressof(ctypes.c_char.from_buffer(data))
    while pos < len(data):
      size = lib.ZSTD_findFrameCompressedSize(address + pos, len(data) - pos)
      if lib.ZSTD_isError(size) or size == 0:
        break
      try:
        # Validate compressed contents too, using bounded decompression memory.
        with zstandard.ZstdDecompressor().stream_reader(io.BytesIO(data[pos:pos + size])) as stream:
          while stream.read(1024 * 1024):
            pass
      except zstandard.ZstdError:
        break
      pos += size
  return pos


def recover_latest(log_root: Path, backup_root: Path) -> list[dict]:
  if not log_root.is_dir():
    return []
  segments = [(m.group(1), int(m.group(2)), p) for p in log_root.iterdir()
              if p.is_dir() and not p.is_symlink() and (m := SEGMENT.fullmatch(p.name))]
  if not segments:
    return []
  latest = max(route for route, _, _ in segments)
  candidates = sorted((part, p) for route, part, p in segments if route == latest)[-2:]
  recovery = backup_root / (time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8])
  changes = []

  def preserve(path: Path) -> Path:
    destination = recovery / path.parent.name / path.name
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, destination)
    with destination.open("rb") as copied:
      os.fsync(copied.fileno())
    sync_dir(destination.parent)
    sync_dir(recovery)
    sync_dir(backup_root)
    sync_dir(backup_root.parent)
    return destination

  for _, segment in candidates:
    for name in ("rlog.zst", "qlog.zst", "rlog.zst.tmp", "qlog.zst.tmp"):
      path = segment / name
      if not path.is_file() or path.is_symlink():
        continue
      size = path.stat().st_size
      prefix = valid_prefix(path)
      if 0 < prefix < size:
        saved = preserve(path)
        with path.open("r+b") as target:
          target.truncate(prefix)
          os.fsync(target.fileno())
        changes.append({"file": str(path), "size": size, "recovered": prefix, "backup": str(saved)})
      if prefix > 0 and name.endswith(".tmp") and not path.with_suffix("").exists():
        path.rename(path.with_suffix(""))
        changes.append({"published": str(path.with_suffix(""))})
    entries = list(segment.iterdir())
    # An empty directory contains no recoverable drive data. Keep it intact in
    # the backup rather than presenting a fictitious segment in the route.
    if entries and all(p.is_file() and not p.is_symlink() and p.stat().st_size == 0 and
                       p.name in {"rlog.zst", "qlog.zst", "rlog.zst.tmp", "qlog.zst.tmp", "rlog.lock"}
                       for p in entries):
      recovery.mkdir(parents=True, exist_ok=True)
      destination = recovery / segment.name
      segment.rename(destination)
      sync_dir(recovery)
      sync_dir(backup_root)
      changes.append({"empty_segment_saved": str(destination)})
    else:
      sync_dir(segment)
    sync_dir(log_root)
  if changes:
    recovery.mkdir(parents=True, exist_ok=True)
    manifest = recovery / "recovery.json"
    with manifest.open("w") as f:
      json.dump(changes, f, indent=2)
      f.flush()
      os.fsync(f.fileno())
    sync_dir(recovery)
    sync_dir(backup_root)
  return changes


def recover_safely(log_root: Path, backup_root: Path) -> None:
  try:
    changes = recover_latest(log_root, backup_root)
    if changes:
      print("LOGGER_RECOVERY " + json.dumps(changes), flush=True)
  except Exception as exc:
    # Recovery failure must never prevent recording the next drive.
    print(f"LOGGER_RECOVERY_FAILED {exc!r}", flush=True)


def run_logger(command: list[str], log_root: Path, backup_root: Path) -> int:
  process: subprocess.Popen | None = None

  def forward_signal(signum, _frame) -> None:
    if process is not None and process.poll() is None:
      process.send_signal(signum)

  previous_handlers = {
    signum: signal.signal(signum, forward_signal)
    for signum in (signal.SIGINT, signal.SIGTERM)
  }
  try:
    process = subprocess.Popen(command, env={**os.environ, "LOGGERD_NO_ENCODERS": "1"})
    return process.wait()
  finally:
    for signum, handler in previous_handlers.items():
      signal.signal(signum, handler)
    # Manager normally lets loggerd close cleanly. If Android interrupted the
    # final zstd frame anyway, repair it before the completed route is read.
    recover_safely(log_root, backup_root)


def main() -> None:
  log_root = Path(os.environ.get("LOG_ROOT", "/data/media/0/realdata"))
  backup_root = Path(os.environ.get("LOGGER_RECOVERY_ROOT", "/data/openpilot/device-backups/logger-recovery"))
  recover_safely(log_root, backup_root)
  raise SystemExit(run_logger(["./loggerd"], log_root, backup_root))


if __name__ == "__main__":
  main()
