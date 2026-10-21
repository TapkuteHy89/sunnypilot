from pathlib import Path

from openpilot.sunnypilot.sunnylink import api


def test_sunnylink_uses_independent_key_pair(tmp_path: Path, monkeypatch):
  private_key = tmp_path / "id_rsa"
  public_key = tmp_path / "id_rsa.pub"
  private_key.write_text("sunny-private")
  public_key.write_text("sunny-public")
  monkeypatch.setattr(api, "SUNNYLINK_PRIVATE_KEY", private_key)
  monkeypatch.setattr(api, "SUNNYLINK_PUBLIC_KEY", public_key)

  assert api.SunnylinkApi.get_key_pair() == ("RS256", "sunny-private", "sunny-public")


def test_sunnylink_key_pair_falls_back_to_comma(tmp_path: Path, monkeypatch):
  monkeypatch.setattr(api, "SUNNYLINK_PRIVATE_KEY", tmp_path / "missing-private")
  monkeypatch.setattr(api, "SUNNYLINK_PUBLIC_KEY", tmp_path / "missing-public")
  monkeypatch.setattr(api.BaseApi, "get_key_pair", staticmethod(lambda: ("RS256", "comma-private", "comma-public")))

  assert api.SunnylinkApi.get_key_pair() == ("RS256", "comma-private", "comma-public")
