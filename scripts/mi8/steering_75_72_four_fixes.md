# Mi 8 / BAIC X7: -75/+72 steering and four follow-up fixes

Status: V3 installed successfully on 2026-10-21. Live CAN confirmed Park,
brake held, 0 km/h and IsOffroad=1 before installation and application flashing.
Both native binaries were built on the Mi 8 for ARM64. All 96 device tests
passed. Panda's application signature matches the selected firmware; faults
remain zero. The service, manager and pandad restarted successfully, and all
twelve installed source checksums match the deployment manifest.
Camera calibration is unchanged. Road validation remains pending.

V1 had rolled back after a missing Qualcomm compatibility-header build error.
V2 stopped before runtime changes at the initial CAN gate. V3 includes the
validated Mi 8 header set independently of earlier deployment directories.

## Candidate changes

1. Steering raw request envelope is -75 through +72. Controller scaling,
   CAN serialization, modern safety, and deployed legacy F4 safety source
   agree. Controller ramp remains 3 and safety delta remains 5. Driver
   override, freshness, checksum, heartbeat, and vehicle gates are retained.
2. The Mi 8 logger supervisor explicitly sets `LOGGERD_NO_ENCODERS=1`.
   Native loggerd uses normal time rotation in that mode without emitting
   the false missing-encoder/camera warning. Other hardware retains its
   existing missing-camera warning.
3. Loggerd closes its streams before messaging cleanup. The supervisor waits
   for native loggerd and recovers incomplete final Zstd frames after exit as
   well as before the next start. Original files are preserved under
   `/data/openpilot/device-backups/logger-recovery`. Recovery cannot recreate
   bytes that were never written before an abrupt power loss.
4. Broadcom orientation recovery requires ten consecutive acceptable fixes
   with a large course disagreement. Missing/invalid fixes and gaps over one
   second clear the consecutive counter. Other receivers retain their prior
   four-sample orientation threshold. Sustained valid disagreement still
   resets the filter.
5. Mi 8 manager sets `SKIP_FW_QUERY=1`. Passive CAN fingerprinting remains
   active; unsupported active firmware/ISO-TP queries are skipped.

## Legacy firmware candidate

Selected source: `android-port/mi8-baic-x7-production/staging/x7-steering-75-72-four-fixes/legacy-exact-build`.
Build: `make -C board bin`, application only, preserving the bootstub. The
unmodified -72/+72 baseline reproduced the installed firmware byte for byte:
38280 bytes, SHA-256 `aaa189163c391ac2bc4d399d7dfb3c3869a0a3f72e56298f9364e2d34c213b4d`.
The selected source differs from that verified baseline only in the two
negative-request constants (-72 to -75). All other firmware behavior is retained.

- File: `board/obj/panda.bin`
- Size: 38280 bytes
- SHA-256: `19b17ddf5059faee7ef8a9bdbf87fbf708a0b240d52d7dd0a9f66acc4866345f`
- Legacy safety tests: 44 passed, including directional boundary checks.
- Modern safety, BAIC controller, and logger recovery tests: 96 passed and
  73 subtests passed.
- Changed Python files pass Ruff; touched tracked diffs pass whitespace checks.
- Linux x86_64 loggerd, locationd, and the native GPS regression executable
  build successfully. Native GPS tests pass for the ten-fix Broadcom debounce,
  unchanged four-fix u-blox behavior, and interrupted bad-course runs.
- A Mi 8 manager startup check confirms firmware probing is skipped, the
  supervised logger is selected, and encoderd remains disabled.

The earlier `opendbc-x7-f4-panda-src` tree lacks the deployed independent
ACC/MADS contract. Its firmware must not be selected for this deployment.
The initial 37372-byte production-source candidate was also not selected:
its header contained additional cleanup changes. Do not substitute it for
the verified 38280-byte minimal-change artifact above. The flasher validates
the connected Panda signature against the old firmware before flashing and
checks the new firmware signature afterward.

## Device verification

Firmware provenance was verified by reproducing the previous installed image.
V3 built loggerd and locationd for ARM64 on the Mi 8 with its bundled
MI8_KERNEL_HEADERS; no local x86_64 runtime binaries were deployed.
The native build, architecture report, 96-test result, before-flash CAN gate,
firmware signature/health report and original sources/binaries are preserved
under `/data/openpilot/device-backups/x7-steering-75-72-four-fixes-v3`.
Deployment stage: `/data/local/tmp/x7-steering-75-72-four-fixes-v3`.
Post-install live CAN again confirmed Park, brake held, 0 km/h and no Panda
faults. The startup journal contained no matched traceback, assertion,
segmentation fault, fatal, ISO-TP, or missing-camera warning at this offroad
check. Onroad-only processes still require a subsequent drive for validation.

Before runtime writes or flashing, verify `IsOffroad=1`, live gear P, brake
held, and 0 km/h. Create a transactional backup in
`/data/openpilot/device-backups`, including a verified rollback firmware.
Use only the application flash path. Verify firmware identity, Panda health,
runtime hashes, and service startup after installation.

The next drive must establish steering saturation, GPS reset rate, clean
logger shutdown, and startup fingerprint behavior. Passing local tests is
not proof that these road symptoms have disappeared.
