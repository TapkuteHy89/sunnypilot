#!/usr/bin/env bash
set -euo pipefail

ROOT=/data/openpilot-next
STAGE=/data/local/tmp/mi8-live-camera-calibration-20261021-v1
BACKUP=/data/openpilot/device-backups/mi8-live-camera-calibration-20261021-v1
PYTHON=$ROOT/.venv/bin/python
PYTEST=/usr/local/venv/bin/pytest
LEGACY_API=/data/local/tmp/x7-driver-gas-state5-deploy-20261018-v1/legacy-api
CALIBRATION_PARAM=/data/params/d/CalibrationParams
LEGACY_EXTRINSICS=/data/mi8_wide_from_device_euler.txt

FILES=(
  openpilot/selfdrive/modeld/mi8_camera.py
  openpilot/selfdrive/modeld/modeld.py
  openpilot/selfdrive/modeld/fill_model_msg.py
  openpilot/sunnypilot/modeld_v2/modeld.py
  openpilot/sunnypilot/modeld_v2/fill_model_msg.py
  openpilot/selfdrive/modeld/tests/test_mi8_camera.py
)

service_was_active=0
state_changed=0
success=0

restore_all() {
  set +e
  sudo -n systemctl stop comma.service >/dev/null 2>&1 || true
  while IFS= read -r relative; do
    test -n "$relative" || continue
    if test -f "$BACKUP/source/$relative"; then
      install -D -o comma -g comma -m 0644 "$BACKUP/source/$relative" "$ROOT/$relative"
    fi
  done < "$BACKUP/file-list.txt"

  if test -f "$BACKUP/params/CalibrationParams"; then
    install -o comma -g comma -m 0600 "$BACKUP/params/CalibrationParams" "$CALIBRATION_PARAM"
  else
    rm -f -- "$CALIBRATION_PARAM"
  fi
  if test -f "$BACKUP/legacy/mi8_wide_from_device_euler.txt"; then
    install -o comma -g comma -m 0644 "$BACKUP/legacy/mi8_wide_from_device_euler.txt" "$LEGACY_EXTRINSICS"
  else
    rm -f -- "$LEGACY_EXTRINSICS"
  fi
  sync
  if test "$service_was_active" -eq 1; then
    sudo -n systemctl start comma.service || true
  fi
}

finish() {
  rc=$?
  trap - EXIT
  if test "$success" -eq 0 && test "$state_changed" -eq 1; then
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
test -f "$STAGE/reset_mi8_camera_calibration.py"
for relative in "${FILES[@]}"; do
  test -f "$STAGE/app/$relative"
done

if systemctl is-active --quiet comma.service; then
  service_was_active=1
  sudo -n systemctl stop comma.service
fi
test "$(cat /data/params/d/IsOffroad 2>/dev/null)" = 1
PYTHONPATH="$LEGACY_API" "$PYTHON" "$STAGE/preflight.py" > "$STAGE/preflight.json"

mkdir -p "$BACKUP/source" "$BACKUP/params" "$BACKUP/legacy"
cp -a "$STAGE/preflight.json" "$BACKUP/preflight.json"
printf '%s\n' "${FILES[@]}" > "$BACKUP/file-list.txt"
for relative in "${FILES[@]}"; do
  mkdir -p "$BACKUP/source/$(dirname "$relative")"
  cp -a "$ROOT/$relative" "$BACKUP/source/$relative"
done
test ! -f "$CALIBRATION_PARAM" || cp -a "$CALIBRATION_PARAM" "$BACKUP/params/CalibrationParams"
test ! -f "$LEGACY_EXTRINSICS" || cp -a "$LEGACY_EXTRINSICS" "$BACKUP/legacy/mi8_wide_from_device_euler.txt"

state_changed=1
for relative in "${FILES[@]}"; do
  install -D -o comma -g comma -m 0644 "$STAGE/app/$relative" "$ROOT/$relative"
  test "$(sha256sum "$ROOT/$relative" | cut -d' ' -f1)" = "$(sha256sum "$STAGE/app/$relative" | cut -d' ' -f1)"
done
rm -f -- "$LEGACY_EXTRINSICS"

cd "$ROOT"
export PYTHONPATH="$ROOT:$ROOT/openpilot:$ROOT/opendbc_repo"
export PYTHONPYCACHEPREFIX="$STAGE/pycache"
"$PYTHON" -m py_compile "${FILES[@]}" "$STAGE/reset_mi8_camera_calibration.py"
"$PYTEST" -q openpilot/selfdrive/modeld/tests/test_mi8_camera.py \
  openpilot/selfdrive/locationd/test/test_calibrationd.py > "$STAGE/test-results.txt"
"$PYTHON" "$STAGE/reset_mi8_camera_calibration.py" > "$STAGE/calibration-reset.json"

sha256sum "${FILES[@]}" > "$STAGE/installed-sha256.txt"
cp -a "$STAGE/test-results.txt" "$STAGE/calibration-reset.json" "$STAGE/installed-sha256.txt" "$BACKUP/"
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
test ! -e "$LEGACY_EXTRINSICS"
success=1
printf 'Mi 8 live camera calibration deployment complete\n'
