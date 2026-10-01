# BUGFIX — uninitialized IMU ring-buffer indices cause segfaults and irreproducible IMU fusion (2026-10-02)

## The bug
`apps/scan_matching_odometry_nodelet.cpp`, class `ScanMatchingOdometryNodelet`:

```cpp
ScanMatchingOdometryNodelet() {}     // empty ctor, no member init
...
int imuPointerFront;                 // UNINITIALIZED
int imuPointerLast;                  // UNINITIALIZED
double imuTime[imuQueLength];        // imuQueLength = 200
float  imuRoll[imuQueLength];
float  imuPitch[imuQueLength];
```

Neither index is initialized anywhere (the only assignments are the two
increments at the imu_callback and inside transformUpdate). `transformUpdate()`
— called only when `enable_imu_fusion=true` — then indexes the 200-element
arrays directly with `imuPointerFront`:

```cpp
if (imuPointerLast >= 0) {
  while (imuPointerFront != imuPointerLast) {
    if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront]) break;
    imuPointerFront = (imuPointerFront + 1) % imuQueLength;
  }
  if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront]) {
    imuRollLast  = imuRoll[imuPointerFront];
    ...
```

## Two distinct failure modes
1. **Crash.** If the indeterminate `imuPointerFront` is out of range, the array
   access is out of bounds -> SIGSEGV. The segfault kills the *nodelet manager*,
   which silently orphans the three `nodelet load` clients. Observed symptom is
   NOT an error message but a "hang": the pipeline stops producing
   `/filtered_points` and `/odom`, bag playback keeps advancing, and all
   remaining nodelet threads sit idle at 0% CPU. Confirmed via
   `dmesg`: `nodelet[...]: segfault ... in libscan_matching_odometry_nodelet.so`
   (16 occurrences in a single boot during this investigation).
2. **Silent corruption.** If the garbage value happens to land in range, there is
   no crash, but the IMU interpolation reads arbitrary/stale ring-buffer entries
   instead of the samples bracketing the scan time. The fusion then blends the
   odometry orientation toward essentially wrong IMU values, in a way that varies
   run to run with heap layout.

`imuPointerLast` is also uninitialized: `if (imuPointerLast >= 0)` can admit
garbage, and `imuPointerLast = (imuPointerLast + 1) % imuQueLength` yields a
NEGATIVE index in C++ if the garbage is negative -> out-of-bounds WRITE in
`imu_callback`.

## Fix
Standard LOAM-style initialization (`Last = -1` is already the sentinel the code
tests with `imuPointerLast >= 0`):

```cpp
int imuPointerFront = 0;
int imuPointerLast  = -1;
```

## Verification
- Before fix: `enable_imu_fusion=true` reproducibly killed the manager
  (segfault) across a fresh container, the original upstream code, a clean
  `rm -rf build devel` rebuild, and a droplet reboot. `enable_imu_fusion=false`
  never crashed.
- After fix: IMU fusion runs normally (`/filtered_points` 12.4 Hz, `/odom`
  12.2 Hz, manager alive), no new segfaults.

## Impact on earlier results — IMPORTANT
All prior IMU-fusion experiments (exp-01 uniform-ratio, exp-02 ratio sweep,
garden validation, and the exp-03 per-axis runs) executed this code path with
uninitialized indices. Runs that did not crash were still subject to failure
mode 2 (reading arbitrary IMU samples). Therefore:

- The IMU-fusion numbers in exp01/exp02/garden_validation are NOT trustworthy
  and must be regarded as provisional until re-run on the fixed build.
- This very plausibly explains the irreproducibility that blocked exp-03: the
  identity control `(r_roll, r_pitch) = (0.1, 0.1)` failed to reproduce the
  earlier uniform `r=0.1` result (roll 3.04 vs 1.31). That was not run-to-run
  noise in the algorithm; it is consistent with the IMU index being garbage.
- The non-IMU results (M1-M3.7 baseline reproduction, the CPU-throughput
  diagnosis, the roll-dominance decomposition of the IMU-OFF baseline) are
  unaffected, since they never call `transformUpdate()`.

## Next steps
1. Re-establish the IMU-fusion reference on the fixed build: uniform r=0.1,
   repeated, to confirm reproducibility.
2. Only then resume exp-03 (per-axis roll/pitch), re-running the 2x2 factorial.
3. Consider reporting upstream (zhuge2333/4DRadarSLAM) — this is a latent
   crash + silent-corruption bug in the released code.
