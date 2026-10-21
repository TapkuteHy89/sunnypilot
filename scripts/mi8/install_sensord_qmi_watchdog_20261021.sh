#!/bin/bash
set -euo pipefail

ROOT=/data/openpilot-next
STAGE=/data/local/tmp/route109-analysis
BACKUP=/data/openpilot/device-backups/sensord-qmi-watchdog-20261021-v1
TARGET="$ROOT/openpilot/system/sensord/sensord_qmi.py"
CANDIDATE="$STAGE/sensord_qmi_candidate.py"
PREFLIGHT="$STAGE/deploy_preflight_offroad_can.py"
TEMP_TARGET="$ROOT/openpilot/system/sensord/.sensord_qmi.py.codex-install-20261021"
PYTHON="$ROOT/.venv/bin/python"

OLD_SHA=189f17c9824671d314e5984d74bd8636010d38ba834b3f68b1df68d21528397a
NEW_SHA=e94a08bb66bba041f56cc3710582fbdc5ffd0b686b0c9073093e6b2def2f204e

verify_hash() {
  local expected=$1
  local path=$2
  printf '%s  %s\n' "$expected" "$path" | sha256sum -c - >/dev/null
}

source_changed=0
rollback() {
  local rc=$?
  trap - EXIT
  if test "$source_changed" -eq 1; then
    install -o comma -g comma -m 0644 "$BACKUP/sensord_qmi.py.before" "$TEMP_TARGET"
    mv -f "$TEMP_TARGET" "$TARGET"
    sync
  fi
  exit "$rc"
}
trap rollback EXIT

test "$(tr -d '\n' < /data/params/d/IsOffroad)" = 1
test -f "$BACKUP/sensord_qmi.py.before"
test -f "$BACKUP/sensord_qmi.py.candidate"
test -f "$CANDIDATE"
test -f "$PREFLIGHT"
test ! -e "$TEMP_TARGET"

verify_hash "$OLD_SHA" "$TARGET"
verify_hash "$OLD_SHA" "$BACKUP/sensord_qmi.py.before"
verify_hash "$NEW_SHA" "$BACKUP/sensord_qmi.py.candidate"
verify_hash "$NEW_SHA" "$CANDIDATE"

cd "$ROOT"
PYTHONPATH="$ROOT:$ROOT/opendbc_repo" timeout 12 "$PYTHON" "$PREFLIGHT" > "$BACKUP/preflight.json"

install -o comma -g comma -m 0644 "$CANDIDATE" "$TEMP_TARGET"
verify_hash "$NEW_SHA" "$TEMP_TARGET"
mv -f "$TEMP_TARGET" "$TARGET"
source_changed=1
sync

verify_hash "$NEW_SHA" "$TARGET"
PYTHONPATH="$ROOT" PYTHONPYCACHEPREFIX=/data/local/tmp/route109-analysis/pycache \
  "$PYTHON" -m py_compile "$TARGET"
test "$(tr -d '\n' < /data/params/d/IsOffroad)" = 1
! pgrep -f '^openpilot\.system\.sensord\.sensord($| )' >/dev/null

SOURCE_HASH="$NEW_SHA" BACKUP_PATH="$BACKUP" "$PYTHON" - <<'PY'
import json
import os
from pathlib import Path

backup = Path(os.environ["BACKUP_PATH"])
result = {
  "pass": True,
  "source_hash": os.environ["SOURCE_HASH"],
  "backup": str(backup),
  "preflight": json.loads((backup / "preflight.json").read_text()),
  "manager_restarted": False,
  "activation": "sensord starts from the new source on the next onroad transition",
}
(backup / "result.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
PY

source_changed=0
trap - EXIT
