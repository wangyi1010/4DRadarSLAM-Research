# exp-01 result — IMU orientation fusion halves front-end rotation error (2026-10-01)

First algorithmic experiment. Hypothesis from exp01_rotation_error_localization.md:
the residual ~2x rotation gap vs paper is a front-end scan-matching problem, and
IMU orientation fusion (hard-disabled in the released launch) is the first lever.

## Setup
Runtime launch variant `m3_runtime/radar_graph_slam_imufusion.launch`: identical
to headless_noplayer but with `enable_imu_fusion:=true` (imu_fusion_ratio=0.1,
the released default value). Repo source NOT modified.
Config: dyn+ego+LC, 0.5x, cp sequence. 5421 /odom, 441 keyframes.

## Result: rotation error (front-end /odom, rot deg/m)
| subtraj (m) | Full baseline | +IMU fusion | change |
|---:|---:|---:|---:|
| 24.6  | 0.1278 | 0.0837 | -34% |
| 49.2  | 0.0851 | 0.0462 | -46% |
| 73.8  | 0.0683 | 0.0315 | -54% |
| 98.3  | 0.0557 | 0.0231 | -59% |
| 122.9 | 0.0426 | 0.0177 | -58% |

At 49 m the FE relative rotation error (0.0462) now matches the paper (0.045).

## The tradeoff
| Metric | Full baseline | +IMU fusion |
|---|---:|---:|
| BE ATE | 0.892 m | 1.041 m |
| FE ATE | 1.797 m | 2.412 m |
| FE RE49 trans% | 4.63 | 4.86 |
| FE RE49 rot deg/m | 0.0851 | **0.0462** |
| FE ATE rot (abs) | 4.49 deg | 6.69 deg |
| BE RE49 rot deg/m | 0.0872 | 0.0651 |

## Interpretation
- IMU fusion sharply reduces LOCAL (relative) rotational jitter -> paper-level.
- It simultaneously INCREASES absolute rotation error (ATE rot 4.49 -> 6.69 deg)
  and degrades translation (ATE 0.89 -> 1.04 m).
- Signature = gyro provides excellent short-term angular rate (kills high-freq
  jitter that relative error measures) but injects a slowly accumulating yaw
  bias (raises absolute rotation, which ATE measures). The fixed
  imu_fusion_ratio=0.1 also over-trusts IMU for translation-coupled terms.
- This likely explains the released launch's `enable_imu_fusion=false`
  ("bad effect"): the authors saw the ATE regression and disabled it, without
  exploiting its large rotational-jitter benefit.

## The opening for a contribution
The two error components now point in opposite directions under the released
fusion scheme:
- rotation wants MORE IMU trust,
- translation wants LESS.
A single fixed ratio cannot optimize both. Candidate contributions:
1. Decouple the fusion: apply IMU orientation only to the rotational part of the
   scan-match prior, leave translation to radar.
2. Adaptive / online IMU-trust weighting (e.g. by ego-velocity confidence or
   scan-match residual).
3. Explicit gyro-bias estimation in the graph (bias state per keyframe) instead
   of a fixed blend.

## Next experiment (exp-02)
Sweep `imu_fusion_ratio` in {0.0 (=off, control), 0.05, 0.1, 0.2, 0.4} at 0.5x,
record FE ATE / FE RE49 rot / BE ATE, and plot the rotation-vs-translation
frontier. This quantifies the tradeoff and tests whether a better fixed ratio
already beats both the paper's rotation and our baseline's ATE.

## Data
notes/exp01_imufusion/: RPG saved_results (FE+BE) + backend trajectory.
