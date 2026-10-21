#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="19.9"
fi

export STAGING_ROOT="/data/safe_staging"

if [ -f /TICI ]; then
  # Mi 8 direct CSL/IFE camera and legacy AGNOS compatibility.
  export NO_DM=1
  export NO_FAN=1
  export NO_LOCATIOND_TEMP=1
  export MI8_KERNEL_HEADERS="$DIR/third_party/linux/include"

  # Validated IMX363 20 Hz profile used by the production Mi 8 port.
  export IMX363_BINNED=1
  export IMX363_20FPS=1
  export IMX363_AE_RESPONSE_SECONDS=0.35
  export IMX363_AE_GREY_DEADBAND=0.045
  export IMX363_DIGITAL_GAIN=1

  # Avoid allocator fragmentation on the memory-constrained phone image.
  export MALLOC_ARENA_MAX=2
  export MALLOC_TRIM_THRESHOLD_=65536
  export MALLOC_MMAP_THRESHOLD_=65536
fi
