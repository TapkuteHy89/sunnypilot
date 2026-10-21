#!/usr/bin/env bash
set -euo pipefail

ROOT=/data/openpilot-next
STAGE=/data/local/tmp/baic-torque-sync-20261021-v1
BACKUP=/data/openpilot/device-backups/baic-torque-sync-20261021-v1
PYTHON=$ROOT/.venv/bin/python
PYTEST=/usr/local/venv/bin/pytest
LEGACY_API=/data/local/tmp/x7-driver-gas-state5-deploy-20261018-v1/legacy-api

FILES=(
  openpilot/sunnypilot/selfdrive/controls/lib/baic_torque.py
  openpilot/sunnypilot/selfdrive/controls/lib/latcontrol_torque_v0.py
  openpilot/selfdrive/controls/lib/latcontrol_torque.py
  openpilot/selfdrive/controls/tests/test_latcontrol_torque_buffer.py
)

service_was_active=0
source_changed=0
success=0

restore_all() {
  set +e
  sudo -n systemctl stop comma.service >/dev/null 2>&1 || true
  while IFS= read -r relative; do
    test -n "$relative" || continue
    if test -f "$BACKUP/source/$relative"; then
      install -D -o comma -g comma -m 0644 "$BACKUP/source/$relative" "$ROOT/$relative"
    elif test -f "$BACKUP/missing-source/$relative"; then
      rm -f -- "$ROOT/$relative"
    fi
  done < "$BACKUP/file-list.txt"
  sync
  if test "$service_was_active" -eq 1; then
    sudo -n systemctl start comma.service || true
  fi
}

finish() {
  rc=$?
  trap - EXIT
  if test "$success" -eq 0 && test "$source_changed" -eq 1; then
    restore_all
  elif test "$success" -eq 0 && test "$service_was_active" -eq 1; then
    sudo -n systemctl start comma.service || true
  fi
  exit "$rc"
}
trap finish EXIT

test "$(cat /data/params/d/IsOffroad 2>/dev/null)" = 1
test ! -e "$BACKUP"
test -x "$PYTHON"
test -x "$PYTEST"
test -d "$LEGACY_API/panda"
test -f "$STAGE/preflight.py"
for relative in "${FILES[@]}"; do
  test -f "$STAGE/app/$relative"
done

if systemctl is-active --quiet comma.service; then
  service_was_active=1
  sudo -n systemctl stop comma.service
fi
test "$(cat /data/params/d/IsOffroad 2>/dev/null)" = 1

PYTHONPATH="$LEGACY_API" "$PYTHON" "$STAGE/preflight.py" > "$STAGE/preflight.json"

mkdir -p "$BACKUP/source" "$BACKUP/missing-source"
cp -a "$STAGE/preflight.json" "$BACKUP/preflight.json"
printf '%s\n' "${FILES[@]}" > "$BACKUP/file-list.txt"
for relative in "${FILES[@]}"; do
  if test -f "$ROOT/$relative"; then
    mkdir -p "$BACKUP/source/$(dirname "$relative")"
    cp -a "$ROOT/$relative" "$BACKUP/source/$relative"
  else
    mkdir -p "$BACKUP/missing-source/$(dirname "$relative")"
    : > "$BACKUP/missing-source/$relative"
  fi
done

source_changed=1
for relative in "${FILES[@]}"; do
  install -D -o comma -g comma -m 0644 "$STAGE/app/$relative" "$ROOT/$relative"
  test "$(sha256sum "$ROOT/$relative" | cut -d' ' -f1)" = "$(sha256sum "$STAGE/app/$relative" | cut -d' ' -f1)"
done

cd "$ROOT"
export PYTHONPATH="$ROOT:$ROOT/openpilot:$ROOT/opendbc_repo"
export PYTHONPYCACHEPREFIX="$STAGE/pycache"
"$PYTHON" -m py_compile "${FILES[@]}"
"$PYTEST" -q openpilot/selfdrive/controls/tests/test_latcontrol_torque_buffer.py > "$STAGE/test-results.txt"

sha256sum "${FILES[@]}" > "$STAGE/installed-sha256.txt"
cp -a "$STAGE/test-results.txt" "$STAGE/installed-sha256.txt" "$BACKUP/"
sync

if test "$service_was_active" -eq 1; then
  sudo -n systemctl start comma.service
  for _ in $(seq 1 30); do
    systemctl is-active --quiet comma.service && pgrep -f './manager.py' >/dev/null && break
    sleep 1
  done
  systemctl is-active --quiet comma.service
  pgrep -f './manager.py' >/dev/null
fi

test "$(cat /data/params/d/IsOffroad 2>/dev/null)" = 1
success=1
printf 'BAIC torque V0/V1 synchronization deployment complete\n'
