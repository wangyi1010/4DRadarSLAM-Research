# exp-02 — IMU fusion ratio sweep (2026-10-01)

Sweep `imu_fusion_ratio` r at 0.5x, Full config (dyn+ego+LC), cp sequence.
r=0 is enable_imu_fusion:=false (M3.7 Full). r=0.1 is exp-01. All ~5421 /odom,
~440 keyframes (frame coverage identical across runs).

## Frontier table (RPG official, SE(3))

| r | FE ATE (m) | BE ATE (m) | RE49 trans% | RE49 rot deg/m | FE abs rot (deg) |
|---:|---:|---:|---:|---:|---:|
| 0.000 | 1.80 | 0.89 | 4.63 | 0.0872 | 4.49 |
| 0.025 | 2.20 | 0.88 | 5.17 | 0.0723 | 4.77 |
| 0.050 | 2.45 | 1.01 | 4.90 | 0.0502 | 6.60 |
| 0.100 | 2.41 | 1.04 | 4.86 | 0.0462 | 6.69 |
| 0.200 | 2.79 | 0.91 | 5.25 | 0.0711 | 7.80 |
| Paper +LC | – | 2.35 | 3.02 | 0.045 | – |

## Findings

### 1. Absolute rotation drift is MONOTONIC in IMU trust
FE abs rotation (ATE rot): 4.49 -> 4.77 -> 6.60 -> 6.69 -> 7.80 deg.
Strictly increasing with r. Confirms exp-01's hypothesis: the IMU injects a
slowly accumulating yaw bias; the more we trust it, the larger the absolute
orientation error.

### 2. Local relative rotation is NON-MONOTONIC
RE49 rot deg/m: 0.0872 -> 0.0723 -> 0.0502 -> 0.0462 (min) -> 0.0711.
Improves to paper-level (0.046) at r=0.1, then WORSENS at r=0.2. At high trust
the accumulating bias begins corrupting even short (49 m) windows, so the
local metric degrades again. The useful regime is a narrow band around r=0.05-0.1.

### 3. BE ATE is also non-monotonic, and decoupled from rotation
BE ATE: 0.89 -> 0.88 (min) -> 1.01 -> 1.04 (max) -> 0.91.
The ATE optimum (r=0.025) and the rotation optimum (r=0.1) are at DIFFERENT
ratios. No single fixed r simultaneously achieves BE ATE <= 0.9 m and
rot ~ 0.045 deg/m.

## Interpretation
A single scalar fusion weight cannot separate the two components of the IMU
signal:
- high-frequency angular rate (reduces local rotational jitter -> want MORE IMU),
- low-frequency gyro bias (accumulates as absolute yaw drift -> want LESS IMU).
A fixed ratio trades one against the other. The non-monotonicity in RE49 rot is
the signature of these two effects crossing over as r increases.

## Conclusion -> motivates exp-03
The clean, measured motivation for a decoupled/adaptive fusion:
- estimate and subtract the gyro bias (per-keyframe bias state in the graph), OR
- frequency-separate: use IMU only for the high-frequency rotational increment,
  let the radar/graph own the low-frequency orientation, OR
- adapt r online by scan-match residual or ego-velocity confidence.
Any of these could, in principle, capture the r=0.1 rotation gain (0.046) while
keeping the r=0.025 ATE (0.88) -- which no fixed r achieves.

## Honest scope note
This remains a reproduction+extension finding. Establishing novelty requires a
literature check: radar-inertial SLAM with gyro-bias graph states and
frequency-separated orientation fusion are known techniques in LiDAR/visual-
inertial work; the contribution (if any) would be their specific application and
benefit in this 4D-radar APDGICP pipeline, demonstrated across multiple
sequences, not just cp.

## Data
notes/exp02_imu_sweep/r{0.025,0.05,0.20}/: RPG saved_results (FE+BE).
r=0 and r=0.1 are in m3_7_valid_ablation/ and exp01_imufusion/.
