#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

source "$DIR/launch_env.sh"

function agnos_init {
  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta
  rm -f /data/scons_cache/config.lock

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0

  # Check if AGNOS update is required
  if [ "$(< /VERSION)" != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/openpilot/common/hardware/comma/agnos.py"
    MANIFEST="$DIR/openpilot/system/hardware/comma/agnos.json"
    if "$AGNOS_PY" --verify "$MANIFEST"; then
      sudo reboot
    fi
    while true; do
      "$DIR/openpilot/common/hardware/comma/updater" "$AGNOS_PY" "$MANIFEST"
    done
  fi
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f "$DIR/.git/index.lock"

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The DIR init file has to exist, with a newer modtime than anything in
  #    the DIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if [ -f "${DIR}/.overlay_init" ]; then
    find "${DIR}/.git" -newer "${DIR}/.overlay_init" | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo "${DIR} has been modified, skipping overlay update installation"
    else
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "Valid overlay update found, installing"
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv "$DIR" /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" "$DIR"
          cd "$DIR"
          # releases before the openpilot/ move can't swap AGNOS on newer AGNOS, use ours
          [ ! -L system/hardware/tici/agnos.json ] && /data/safe_staging/old_openpilot/openpilot/common/hardware/comma/agnos.py --verify system/hardware/tici/agnos.json && sudo reboot

          echo "Restarting launch script ${LAUNCHER_LOCATION}"
          unset AGNOS_VERSION
          exec "${LAUNCHER_LOCATION}"
        else
          echo "openpilot backup found, not updating"
          # TODO: restore backup? This means the updater didn't start after swapping
        fi
      fi
    fi
  fi

  # handle pythonpath
  ln -sfn "$(pwd)" /data/pythonpath
  export PYTHONPATH="$PWD"

  # The Mi 8 image predates the dependency set used by current sunnypilot.
  # Keep the migrated runtime isolated from the production system venv so a
  # failed test can be rolled back without leaving mixed Python packages.
  if [ -f /TICI ] && [ -x "$DIR/.venv/bin/python3" ]; then
    export VIRTUAL_ENV="$DIR/.venv"
    export PATH="$VIRTUAL_ENV/bin:$PATH"
  fi

  # raylib 6's comma backend selects an EGL config that the SDM845/Mi 8
  # display stack does not expose. The device image ships a comma raylib 5.5
  # backend which has the same Python API used by the current UI and has been
  # validated as a magic DRM client on this panel. Prefer only those two
  # packages on non-comma /TICI compatibility devices; keep every other
  # dependency in the isolated current-sunnypilot venv.
  if [ -f /TICI ] && [ -r /proc/device-tree/model ] && \
     ! grep -aqi 'comma' /proc/device-tree/model && \
     [ -d /usr/local/venv/lib/python3.12/site-packages/raylib ] && \
     [ -d /usr/local/venv/lib/python3.12/site-packages/pyray ]; then
    MI8_RAYLIB_COMPAT=/tmp/mi8-raylib-py
    rm -rf "$MI8_RAYLIB_COMPAT"
    mkdir -p "$MI8_RAYLIB_COMPAT"
    ln -s /usr/local/venv/lib/python3.12/site-packages/raylib "$MI8_RAYLIB_COMPAT/raylib"
    ln -s /usr/local/venv/lib/python3.12/site-packages/pyray "$MI8_RAYLIB_COMPAT/pyray"
    export PYTHONPATH="$MI8_RAYLIB_COMPAT:$PYTHONPATH"
  fi

  # submodule package symlinks for PYTHONPATH imports on device.
  # on PC these come from editable installs via pyproject.toml / uv.
  ln -sfn msgq_repo/msgq msgq
  ln -sfn opendbc_repo/opendbc opendbc
  ln -sfn rednose_repo/rednose rednose
  ln -sfn teleoprtc_repo/teleoprtc teleoprtc
  ln -sfn tinygrad_repo/tinygrad tinygrad
  # Git symlinks are checked out as text files on the Windows migration host.
  # Restore the generated vehicle catalog link before the Python UI loads it.
  ln -sfn ../../../../opendbc_repo/opendbc/sunnypilot/car/car_list.json \
    openpilot/sunnypilot/selfdrive/car/car_list.json

  # hardware specific init
  # The Mi 8 compatibility image exposes both /AGNOS and /TICI markers, but it
  # is not an AGNOS-updatable comma device. Never enter the slot updater here.
  if [ -f /AGNOS ] && [ ! -f /TICI ]; then
    agnos_init
  fi

  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  # start manager
  cd openpilot/system/manager
  if [ ! -f "$DIR/prebuilt" ]; then
    ./build.py
  fi
  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
