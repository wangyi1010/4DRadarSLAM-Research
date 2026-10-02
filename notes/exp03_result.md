# exp-03 RESULT — per-axis roll/pitch IMU fusion: refuted (2026-10-02)

All numbers below are on the FIXED build (commit 8de7619, uninitialized IMU
ring-buffer indices). Everything measured before that fix is invalid; see
notes/bugfix_imu_pointer_init.md.

Config: cp sequence, dyn OFF, ego OFF, LC ON, rate=0.5, keyframe_delta_trans_back_end=0.5.
2x2 factorial over (imu_fusion_ratio_roll, imu_fusion_ratio_pitch), 3 repeats per
cell, fresh container stop/start before each run. 12/12 runs completed; all runs
5427 odom frames and 430 keyframes; zero segfaults throughout.

## Results (mean +- SD, n=3 per cell)

| cell | FE ATE (m) | BE ATE (m) | RE49 rot (deg/m) | roll (deg) | pitch (deg) |
|---|---:|---:|---:|---:|---:|
| (0, 0) IMU off   | 2.320 +- 0.000 | 1.160 | **0.073 +- 0.000** | **2.593 +- 0.000** | 2.223 +- 0.000 |
| (0.1, 0.1) both  | 2.710 +- 0.033 | 1.163 | 0.074 +- 0.004 | 3.026 +- 0.245 | 1.916 +- 0.229 |
| (0.1, 0) roll    | 2.253 +- 0.026 | 1.150 | 0.133 +- 0.005 | 3.411 +- 0.289 | 5.445 +- 0.109 |
| (0, 0.1) pitch   | 2.577 +- 0.005 | 1.170 | 0.082 +- 0.000 | 3.185 +- 0.004 | **1.566 +- 0.003** |

## Finding 1 — the pipeline is deterministic; all variance is in the IMU path
The (0,0) cell reproduces bit-identically across 3 runs (SD = 0 on every metric).
So scan matching and the pose graph are deterministic at this operating point,
and the spread seen in IMU-on cells (roll SD up to 0.29) originates in the IMU
fusion itself - plausibly which IMU sample lands nearest each scan, which shifts
slightly with message timing. This gives a principled error bar: a roll effect
must exceed roughly +-0.3 deg to be real.

## Finding 2 — per-axis decoupling is INVALID (strong negative interaction)
Effects relative to the (0,0) baseline:

| metric | roll-only | pitch-only | sum of singles | both | interaction |
|---|---:|---:|---:|---:|---:|
| roll     | +0.818 | +0.592 | +1.410 | +0.433 | **-0.977** |
| pitch    | +3.222 | -0.657 | +2.565 | -0.307 | **-2.872** |
| RE49 rot | +0.060 | +0.009 | +0.069 | +0.001 | **-0.068** |

The interaction terms are as large as, or larger than, the main effects. For
pitch the interaction (-2.87) exceeds both singles. For aggregate rotation the
interaction (-0.068) almost exactly cancels the sum of singles (+0.069). The two
weights are therefore NOT independent knobs.

Two concrete demonstrations of coupling:
- Enabling ONLY roll fusion blows pitch error up from 2.22 to **5.45 deg**
  (tight: SD 0.11), and doubles aggregate rotation error (0.073 -> 0.133).
- Enabling ONLY pitch fusion still degrades ROLL (2.59 -> 3.19), even though the
  roll weight is zero.

Mechanism: the IMU supplies a joint, gravity-referenced 2-DOF tilt. The code
decomposes orientation with R2ypr and rebuilds it via
createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw), a coupled yaw-pitch-roll
sequence. Blending one Euler component toward the IMU while leaving the other at
the odometry value reconstructs a rotation that corresponds to neither source;
the inconsistency surfaces in the untouched axis. A 2-DOF gravity constraint does
not decompose into two independent 1-DOF scalar weights.

=> exp-03's central hypothesis (separate r_roll and r_pitch to keep the roll
benefit while dropping the pitch penalty) is REFUTED. The asymmetric cells are
strictly worse than either (0,0) or (0.1,0.1) on aggregate rotation.

## Finding 3 — IMU fusion as implemented gives no net benefit
The IMU-off baseline has the BEST aggregate rotation error of all four cells
(0.073; the others are 0.074, 0.082, 0.133) and the second-best translation.
Enabling fusion trades a modest pitch gain (2.22 -> 1.92/1.57) against a roll
loss (2.59 -> 3.03/3.19) and a translation penalty (FE ATE 2.32 -> 2.71 for the
symmetric cell). Net effect on RE49 rot is +0.001, i.e. nothing.

This DIRECTLY CONTRADICTS the pre-fix exp-01/exp-02/garden conclusion that IMU
fusion roughly halves the roll error. That result was produced by the
uninitialized-index bug and does not survive correct code.

## Status of earlier IMU claims
RETRACTED (all executed transformUpdate() with uninitialized indices):
- exp-01: "IMU fusion halves front-end rotation error" -- not reproducible.
- exp-02: the ratio sweep and its non-monotonic frontier.
- garden_validation: the IMU portions (the dataset/segment handling is still fine).
- exp01_rotation_error_localization: the IMU-ON comparisons. NOTE the IMU-OFF
  decomposition in that note (rotation error is roll-dominated in the baseline)
  does NOT touch transformUpdate and remains valid.

UNAFFECTED: M1-M3.7 baseline reproduction, the CPU-throughput diagnosis
(m3_6_cpu_diagnosis.md), and the M3.7 ablation of dyn/ego/LC.

## What is actually worth keeping from this line of work
1. The bugfix (8de7619): a latent crash + silent-corruption defect in released
   upstream code. This is the substantive contribution.
2. A deterministic baseline and a measured error bar for this pipeline, which is
   what made a trustworthy negative result possible.
3. A clean structural negative result: Euler-component-wise blending of a
   gravity-referenced tilt is not a valid design, with factorial evidence.

## Honest assessment
This is a negative result for the proposed method, not a new SLAM technique. Its
value is reproducibility work: finding the bug, showing the prior IMU claims do
not survive it, and establishing the controls (determinism, error bars,
factorial interaction) needed to make such claims safely. Any future orientation
work here should operate on SO(3) directly (e.g. a relative-rotation or
bias-estimating formulation) rather than per-Euler-axis scalar blending.

## Data
notes/exp03_factorial/: factorial_summary.txt (12 runs), repro3_summary.txt
(3-run reproducibility of uniform r=0.1).
