# exp-03 pre-implementation diagnostic (2026-10-01)

Before changing fusion code, measured WHERE the IMU-fusion rotation benefit and
cost come from, to avoid designing against the wrong mechanism.

## D1 — IMU vs GT roll/pitch offset (scripts/diag_rollpitch.py)
At radar timestamps, IMU orientation (after extrinsicRPY + initial-reference,
exactly as the code applies it) vs GT roll/pitch. Timestamp sweep +/-50 ms.

| seq | roll mean | roll rmse | roll slope | pitch mean | pitch rmse | pitch slope | timing |
|---|---:|---:|---:|---:|---:|---:|---|
| cp     | -0.205 deg | 0.51 | -0.0018 deg/s | -0.259 deg | 0.54 | -0.0002 deg/s | flat |
| garden | -0.404 deg | 0.64 | -0.0050 deg/s | -0.382 deg | 0.55 | -0.0010 deg/s | flat |

Findings:
- IMU roll/pitch is ACCURATE vs GT (RMSE ~0.5-0.6 deg), NO drift (slope ~0),
  timing-insensitive (+/-50 ms barely moves rmse).
- Small near-constant offset that DIFFERS by sequence (cp ~ -0.2, garden ~ -0.4).
  Consistent with reference/calibration handling, not sensor bias.
- The offset (0.2-0.4 deg) is FAR too small to explain the multi-degree rise in
  absolute rotation error seen when raising imu_fusion_ratio. So the ATE-rot
  cost is NOT a simple roll/pitch reference offset.

## D2 — per-axis rotation error over distance (scripts/diag_axis_window.py)
RPG measures rot error over ~49 m (cp) / ~68 m (garden) sub-trajectories.
Decompose that windowed relative-rotation error into roll/pitch/yaw.

cp (49 m windows):
| axis | IMU-off | r=0.10 | change |
|---|---:|---:|---:|
| geodesic | 4.04 | 2.19 | -46% |
| roll | 3.05 | 1.31 | -57% |
| pitch | 2.47 | 1.43 | -42% |
| yaw | 0.98 | 1.01 | +4% |

garden (68 m windows):
| axis | IMU-off | r=0.10 | change |
|---|---:|---:|---:|
| geodesic | 3.98 | 3.36 | -16% |
| roll | 3.25 | 1.37 | -58% |
| pitch | 2.05 | 2.88 | +40% |
| yaw | 1.07 | 1.06 | ~0% |

Findings:
1. Rotation error over distance is ROLL-DOMINATED on both sequences (radar's weak
   vertical/elevation axis). IMU gravity-referenced roll correction fixes it
   robustly: -57% (cp), -58% (garden). This is the real, generalizable benefit.
2. YAW is small (~1 deg) and UNCHANGED by fusion on both. It is NOT the residual.
   => drop the yaw-increment idea (old C3).
3. PITCH is offset-sensitive: improves on cp (-42%), DEGRADES on garden (+40%) at
   r=0.10, exactly where garden's larger IMU pitch offset is over-weighted. This
   is what makes the rotation-optimal ratio sequence-dependent (cp 0.10, garden 0.025).

## Design conclusion -> exp-03 = per-axis roll/pitch fusion
The single scalar r_roll = r_pitch = r is itself the limitation. The two axes
need different weights:
- roll: benefits strongly and robustly -> keep high weight.
- pitch: offset-sensitive -> lower (or zero) weight.
- yaw: untouched (already good).

exp-03 change: split imu_fusion_ratio into (imu_fusion_ratio_roll,
imu_fusion_ratio_pitch). Keep absolute gravity-referenced fusion (NOT relative
increments, which would discard the roll gravity anchor that provides the roll
benefit; NOT online offset estimation, which would absorb odometry error into a
fake IMU offset and erode the anchor).

Key causal test: (r_roll, r_pitch) = (0.10, 0) -- does it retain the ~58% roll
gain on BOTH sequences while removing the pitch penalty and recovering ATE?

Deferred: exp-04 fixed init-window offset compensation (if still needed);
exp-05 move fusion into the APDGICP prior (changes convergence basin; less
controlled, so last).
