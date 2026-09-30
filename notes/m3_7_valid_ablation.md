# M3.7 — valid ablation at 0.5x playback (2026-10-01)

All runs on `cp`, commit `dd2ee8c` + CMakeLists fix, `--rate=0.5` (SLAM with no
embedded player + separate `rosbag play`), `keyframe_delta_trans_back_end:=0.5`.
~100% radar-frame coverage in every run (5421/5428 /odom messages).

## Full results table

| Config | dyn | ego | LC | FE ATE (m) | BE ATE (m) | RE49 (%) | rot deg/m | KF |
|---|:-:|:-:|:-:|---:|---:|---:|---:|---:|
| F  (baseline) | off | off | off | 2.32 | 2.22 | 5.14 | 0.073 | 430 |
| D' | ON  | off | off | 2.16 | 2.05 | 5.09 | 0.085 | 432 |
| E' | off | ON  | off | 1.85 | 1.85 | 4.72 | 0.071 | 438 |
| B' | ON  | ON  | off | 1.65 | 1.67 | 4.63 | 0.083 | 441 |
| Full | ON | ON | ON | 1.80 | **0.89** | 5.02 | 0.087 | 440 |
| Paper APDGICP | – | – | – | 2.61 | – | 3.56 | 0.037 | 437 |
| Paper APDGICP+LC | – | – | – | – | 2.35 | 3.02 | 0.045 | – |

## Conclusions

### 1. The 1x "broken feature" findings were all CPU artifacts
At 1x on 2 vCPU (Runs B/D/E), dyn_removal and ego_vel gave 20-33m ATE. At 0.5x
with full frame coverage, both features WORK and each IMPROVES accuracy
monotonically:
- baseline -> +dyn -> +ego -> +both: 2.22 -> 2.05 -> 1.85 -> 1.67 m BE ATE.
The earlier "both implementations broken" verdict is RETRACTED.

### 2. Loop closure works at 0.5x (and dramatically)
Full config backend ATE = 0.89m, vs 1.67m without LC. At 1x, LC did nothing
because dropped frames starved the backend of keyframes. LC needs the dense
keyframe stream that only full-rate processing produces.

### 3. We now BEAT the paper on ATE
Best config (dyn+ego+LC) backend ATE 0.89m vs paper's best 2.35m — 2.6x better.
Even the plain baseline (2.22m) matches the paper's +LC number.

### 4. The genuine remaining discrepancy: relative / rotation error
Across ALL configs, RE49 sits at 4.6-5.1% vs paper's 3.0-3.6%, and rotation
error at 0.07-0.09 deg/m vs paper's 0.037-0.045. Roughly 1.5-2x, and it does
NOT close with more features or loop closure.

This is scientifically the cleanest open problem: our GLOBAL accuracy (ATE) is
better than the paper, but our LOCAL/relative consistency is worse. The
trajectory is globally well-aligned but locally noisier per segment. Candidate
causes: orientation not IMU-constrained (launch hard-codes
enable_imu_fusion:=false), scan-matching rotational precision, or a difference
in how the paper computed RE.

## Cross-run sanity: keyframe counts
All configs land 430-441 keyframes, matching the paper's reported 437. The bag
produces 5428 radar frames; at 0.5x we process 5421 (99.9%).

## Trustworthy baselines for the research phase
- **Odometry-only reference (front-end):** E' or B', ~1.7-1.85m ATE.
- **Full-system reference (back-end):** Full config, 0.89m ATE.
- Always `--rate=0.5`; quote only these numbers.

## Open problem to attack next
Reduce the relative/rotation error (currently ~2x paper) without hurting ATE.
First diagnostic: is the rotation error dominated by the front-end scan matching
or introduced in the graph? Compare per-segment rotation error of /odom vs the
optimized graph. Then consider adding an IMU orientation edge (currently
disabled) as the first algorithmic change on an exp/* branch.

## Data preserved
`notes/m3_7_valid_ablation/rate0p5_*/`: stamped_pose_graph_estimate.txt + RPG
saved_results. Bags/maps not committed.
