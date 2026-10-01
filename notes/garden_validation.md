# Garden_0 segment validation of the IMU-fusion tradeoff (2026-10-01)

Second-sequence check of the cp exp-01/exp-02 finding. NOT a full-garden
reproduction: the official garden launch plays BOTH 2022-05-13 bags; we ran only
`garden_2022-05-13_0.bag` (3.8 GB, first ~401 s). Absolute numbers are therefore
not comparable to paper-level `garden`; only the qualitative tradeoff structure
is under test.

## Setup
- Bag: garden_0 (SHA 3ec4815256652f8b2222f99b4cc01afabdfd58ba08d57ca1880ba0a365a540b9)
- GT: garden gt_odom.txt (SHA c5c5122e..., 6874 poses, covers both bags; RPG matches bag_0 range)
- 0.5x playback, Full config (dyn+ego+LC), keyframe_delta_trans_back_end:=0.5
- Frame coverage: 4815/4816 radar -> /odom (99.98%); 351 keyframes. CPU-drop confounding excluded.
- Metric length: 67.8 m sub-trajectory (garden preset nearest cp's 49 m).

## Results
| config | FE ATE (m) | BE ATE (m) | RE68 trans% | RE68 rot deg/m | FE abs rot (deg) |
|---|---:|---:|---:|---:|---:|
| IMU off | 0.63 | 0.62 | 3.46 | 0.0569 | 3.03 |
| r=0.025 | 0.77 | 0.75 | 5.01 | 0.0458 | 5.37 |
| r=0.10  | 0.80 | 0.79 | 5.49 | 0.0504 | 7.03 |

## Qualitative replication vs cp
1. ABS ROTATION DRIFT MONOTONIC in IMU trust: 3.03 -> 5.37 -> 7.03 deg.
   Same as cp (4.49 -> 7.80). The cleanest, most robust signal; now confirmed
   on two sequences. (Still "consistent with accumulated IMU orientation bias",
   not proof of gyro yaw bias.)
2. RELATIVE ROTATION NON-MONOTONIC: 0.0569 -> 0.0458 (min) -> 0.0504.
   Same structure as cp (improves to a minimum, then worsens).
3. IMU COSTS GLOBAL/TRANSLATION ACCURACY: BE ATE 0.62 -> 0.79 m;
   RE68 trans 3.46 -> 5.49 %. Same direction as cp.

## The key new result: the optimal ratio is sequence-dependent
- cp rotation-optimal ratio: r = 0.10 (RE49 rot 0.0462)
- garden rotation-optimal ratio: r = 0.025 (RE68 rot 0.0458)
No single fixed imu_fusion_ratio is optimal across both sequences. This is a
stronger argument for adaptive / decoupled fusion than the single-sequence
exp-02 result: a tuned constant that is best on cp is suboptimal on garden.

## Scope / honesty
- Segment validation only (bag_0, not full garden). Two sequences, both NTU.
- Garden baseline is already quite accurate (0.63 m ATE); the IMU rotation
  benefit is smaller in absolute terms than on cp, but the direction/structure
  hold.
- Not a novel contribution yet; this validates that the tradeoff generalizes
  beyond cp, which is the precondition for designing exp-03.

## Environment robustness notes (not findings)
- After ~10 back-to-back launches the container roscore accumulates bad state:
  nodelet bond breaks ("Bond broken, exiting") or preprocessing hangs after the
  first frame. Fix: `docker restart radar` (workspace is bind-mounted, persists).
- Added global param `/bond_disable_heartbeat_timeout=true` to the runtime
  launches to stop sim-time-triggered bond breaks.

## Conclusion
The IMU-fusion local-rotation-vs-global-accuracy tradeoff is NOT cp-specific.
It replicates on garden_0, and the sequence-dependent optimum reinforces the
case for exp-03 (adaptive / decoupled / bias-estimating fusion) over a tuned
fixed ratio.

## Data
notes/garden_validation/garden_imu_{off,r0.025,r0.10}/: RPG saved_results (FE+BE).
