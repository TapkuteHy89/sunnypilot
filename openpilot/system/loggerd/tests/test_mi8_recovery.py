import tempfile
import unittest
from pathlib import Path
from unittest.mock import ANY, MagicMock, patch

import zstandard

from openpilot.system.loggerd.mi8_loggerd import recover_latest, run_logger, valid_prefix


class TestMi8Recovery(unittest.TestCase):
  def test_zero_tail_and_empty_rotation_preserve_originals(self):
    with tempfile.TemporaryDirectory() as tmp:
      root = Path(tmp) / "logs"
      backup = Path(tmp) / "backups"
      segment = root / "000000fc--e12c26b447--7"
      empty = root / "000000fc--e12c26b447--8"
      segment.mkdir(parents=True)
      empty.mkdir()
      compressor = zstandard.ZstdCompressor()
      good = compressor.compress(b"first message") + compressor.compress(b"second message")
      original = good + bytes(2181)
      (segment / "rlog.zst").write_bytes(original)
      (segment / "qlog.zst").write_bytes(good)
      for name in ("rlog.zst", "qlog.zst"):
        (empty / name).touch()
      changes = recover_latest(root, backup)
      self.assertEqual((segment / "rlog.zst").read_bytes(), good)
      self.assertEqual(valid_prefix(segment / "qlog.zst"), len(good))
      self.assertEqual(Path(changes[0]["backup"]).read_bytes(), original)
      self.assertTrue(Path(changes[1]["empty_segment_saved"]).is_dir())
      self.assertFalse(empty.exists())
      self.assertEqual(recover_latest(root, backup), [])

  def test_incomplete_frame_only_loses_unsaved_suffix(self):
    with tempfile.TemporaryDirectory() as tmp:
      root = Path(tmp) / "logs"
      segment = root / "00000001--0123456789--0"
      segment.mkdir(parents=True)
      first = zstandard.ZstdCompressor().compress(b"complete event")
      last = zstandard.ZstdCompressor().compress(b"incomplete event")[:-3]
      path = segment / "rlog.zst"
      path.write_bytes(first + last)
      recover_latest(root, Path(tmp) / "backups")
      self.assertEqual(path.read_bytes(), first)

  def test_publish_complete_header_and_preserve_media(self):
    with tempfile.TemporaryDirectory() as tmp:
      root = Path(tmp) / "logs"
      segment = root / "00000001--0123456789--0"
      segment.mkdir(parents=True)
      header = zstandard.ZstdCompressor().compress(b"init data")
      (segment / "rlog.zst.tmp").write_bytes(header)
      (segment / "qlog.zst").touch()
      (segment / "fcamera.hevc").write_bytes(b"video evidence")
      recover_latest(root, Path(tmp) / "backups")
      self.assertEqual((segment / "rlog.zst").read_bytes(), header)
      self.assertEqual((segment / "fcamera.hevc").read_bytes(), b"video evidence")

  def test_leave_older_routes_untouched(self):
    with tempfile.TemporaryDirectory() as tmp:
      root = Path(tmp) / "logs"
      older = root / "00000001--0123456789--0"
      latest = root / "00000002--0123456789--0"
      older.mkdir(parents=True)
      latest.mkdir()
      good = zstandard.ZstdCompressor().compress(b"event")
      (older / "rlog.zst").write_bytes(good + bytes(100))
      (latest / "rlog.zst").write_bytes(good)
      self.assertEqual(recover_latest(root, Path(tmp) / "backups"), [])
      self.assertEqual((older / "rlog.zst").stat().st_size, len(good) + 100)

  @patch("openpilot.system.loggerd.mi8_loggerd.recover_safely")
  @patch("openpilot.system.loggerd.mi8_loggerd.signal.signal")
  @patch("openpilot.system.loggerd.mi8_loggerd.subprocess.Popen")
  def test_supervisor_waits_and_recovers_after_logger_exit(self, popen, signal_fn, recover):
    process = MagicMock()
    process.wait.return_value = 0
    popen.return_value = process
    signal_fn.return_value = None
    log_root = Path("logs")
    backup_root = Path("backups")

    self.assertEqual(run_logger(["./loggerd"], log_root, backup_root), 0)
    popen.assert_called_once_with(["./loggerd"], env=ANY)
    self.assertEqual(popen.call_args.kwargs["env"]["LOGGERD_NO_ENCODERS"], "1")
    process.wait.assert_called_once_with()
    recover.assert_called_once_with(log_root, backup_root)


if __name__ == "__main__":
  unittest.main()
