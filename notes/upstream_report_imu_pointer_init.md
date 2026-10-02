# Uninitialized IMU ring-buffer indices in ScanMatchingOdometryNodelet

Affects: `apps/scan_matching_odometry_nodelet.cpp` @ dd2ee8c2378630a84361f37331ba14665b203c14
Trigger: any run with `enable_imu_fusion:=true`

## Summary

`imuPointerFront` and `imuPointerLast` are declared without initialization:

```cpp
ScanMatchingOdometryNodelet() {}          // empty constructor
...
int imuPointerFront;                      // never initialized
int imuPointerLast;                       // never initialized
double imuTime[imuQueLength];             // imuQueLength = 200
float  imuRoll[imuQueLength];
float  imuPitch[imuQueLength];
```

They are then used as indices into those fixed-size buffers in
`transformUpdate()`, which is called only when `enable_imu_fusion` is true:

```cpp
if (imuPointerLast >= 0) {
  while (imuPointerFront != imuPointerLast) {
    if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront]) break;
    imuPointerFront = (imuPointerFront + 1) % imuQueLength;
  }
  if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront]) {
    imuRollLast  = imuRoll[imuPointerFront];
    imuPitchLast = imuPitch[imuPointerFront];
  } else {
    int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
    ...
  }
```

Reading an indeterminate value is undefined behaviour, and here it is used
directly as an array index.

## Observed behaviour

Instrumenting the pristine code to print the indices at the first call to
`transformUpdate()` (before any assignment to `imuPointerFront` can have
occurred), across three launches from a freshly started container:

```
launch 1: imuPointerFront=52688  imuPointerLast=30  imuQueLength=200
launch 2: imuPointerFront=0      imuPointerLast=38  imuQueLength=200
launch 3: imuPointerFront=0      imuPointerLast=35  imuQueLength=200
```

`imuPointerFront = 52688` indexes roughly 421 KB past a 200-element array. The
value differs between launches, as expected for an indeterminate read.

This produces two distinct failure modes:

1. **Crash.** When the value is far out of range, the access faults and takes
   down the nodelet manager. In our environment this presented not as an error
   message but as an apparent hang: `/filtered_points` and `/odom` stop, bag
   playback keeps advancing, and the orphaned `nodelet load` clients sit idle at
   0% CPU. The kernel log is explicit:

   ```
   nodelet[...]: segfault at ... in libscan_matching_odometry_nodelet.so[...]
   ```

   We recorded 16 such segfaults in a single boot while running with
   `enable_imu_fusion:=true`. Running the identical configuration with
   `enable_imu_fusion:=false` (which does not call `transformUpdate()`) produced
   none.

2. **Silent incorrect output.** When the value happens to fall within range,
   there is no crash, but the interpolation reads whichever ring-buffer entries
   that index selects rather than the samples bracketing the scan time. The
   fused roll/pitch is then derived from the wrong IMU samples, and the result
   varies between runs with heap layout.

`imuPointerLast` is likewise uninitialized. It is tested with
`if (imuPointerLast >= 0)`, and in `imu_callback` it is advanced with
`imuPointerLast = (imuPointerLast + 1) % imuQueLength`. If the indeterminate
value is negative, that expression remains negative in C++, giving an
out-of-bounds write.

## Fix

Initialize both indices to the convention the surrounding code already assumes
(`-1` is the "no IMU data yet" sentinel that `if (imuPointerLast >= 0)` tests):

```cpp
int imuPointerFront = 0;
int imuPointerLast = -1;
```

Two lines, no behavioural change beyond removing the undefined behaviour.

## Verification

- Before: `enable_imu_fusion:=true` reproduced the crash across a freshly created
  container, a clean `rm -rf build devel` rebuild, and a host reboot.
  `enable_imu_fusion:=false` never crashed.
- After: 15 consecutive runs with IMU fusion enabled (3 reproducibility runs plus
  a 12-run parameter sweep) completed full trajectories, each processing the same
  5427 odometry frames and 430 keyframes, with no further segfaults.

## Notes

- A separate build issue was encountered before this could be exercised at all:
  the `scan_matching_odometry_nodelet` target does not compile/link against
  `src/radar_graph_slam/keyframe.cpp` or the g2o libraries, so the nodelet fails
  to load with an undefined symbol for `radar_graph_slam::KeyFrame::KeyFrame`.
  That is independent of this report and is not included in this change.
- AddressSanitizer was tried first but is not the right instrument here: it does
  not detect uninitialized reads, and an out-of-range index only trips it when it
  clears the enclosing object's allocation. The direct instrumentation above is
  the clearer evidence.
