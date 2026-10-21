#!/usr/bin/env bash

set -e

# Current sunnypilot's raylib UI receives the DRM master fd from magic. The
# legacy Mi 8 production UI uses Weston, so hand the display over before the
# manager starts.
sudo systemctl stop weston 2>/dev/null || true
sudo systemctl unmask magic 2>/dev/null || true
sudo systemctl daemon-reload
# A previous interrupted candidate run may leave magic active while its socket
# has been removed. Stop the owner before clearing the socket so start always
# recreates both the DRM session and /tmp/drmfd.sock.
sudo systemctl stop magic 2>/dev/null || true
sudo rm -f /tmp/drmfd.sock
sudo systemctl start magic

for _ in $(seq 1 40); do
  systemctl is-active --quiet magic && [ -S /tmp/drmfd.sock ] && break
  sleep 0.25
done
if [ ! -S /tmp/drmfd.sock ]; then
  echo "Mi8 display handoff failed: /tmp/drmfd.sock was not created" >&2
  exit 1
fi

# SDM845 performance cluster: direct CSL/IFE camera capture needs the big
# cores online. Keep the same validated policy as the production launcher.
sudo sh -c 'echo 4 > /sys/devices/system/cpu/cpu4/core_ctl/min_cpus' 2>/dev/null || true
sudo sh -c 'echo 4 > /sys/devices/system/cpu/cpu4/core_ctl/max_cpus' 2>/dev/null || true
for c in 4 5 6 7; do
  sudo sh -c "echo 1 > /sys/devices/system/cpu/cpu${c}/online" 2>/dev/null || true
done
sudo sh -c 'echo performance > /sys/devices/system/cpu/cpufreq/policy4/scaling_governor' 2>/dev/null || true

# CamX can create worker threads after camerad starts. Keep all capture
# threads on CPU4+7 without changing the device thermal limits.
(
  while true; do
    sleep 10
    pid=$(pgrep -xo camerad_mi8 2>/dev/null || true)
    if [ -n "$pid" ]; then
      for tid in /proc/"$pid"/task/*; do
        taskset -p 0x90 "${tid##*/}" >/dev/null 2>&1 || true
      done
    fi
  done
) &

# Xiaomi Mi 8: expose the FocalTech touchscreen under both names used by the
# old comma raylib backend and the current UI stack. Do not use a loose
# "touch" grep here: on this image it can select the gpio-keys event instead.
TS_EVT=$(sed -n '/N: Name="fts"/,/^$/ { /Handlers=/ s/.* \(event[0-9]\+\).*/\1/p }' /proc/bus/input/devices | head -1)
if [ -n "$TS_EVT" ] && [ -e "/dev/input/$TS_EVT" ]; then
  sudo umount /dev/input/event2 2>/dev/null || true
  sudo mount --bind "/dev/input/$TS_EVT" /dev/input/event2 2>/dev/null || true
  sudo ln -sfn "../$TS_EVT" /dev/input/by-path/platform-894000.i2c-event
fi

cd /data/openpilot-next
exec ./launch_chffrplus.sh
