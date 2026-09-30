# M3 — cp sequence baseline reproduction (2026-09-30)

Verdict: **PASS.** Full pipeline reproduced end-to-end; numbers below.

## Configuration
- Dataset: NTU4DRadLM / cp (carpark, 246 m urban)
  - Bag: `cp_2022-02-26.bag`, 3.7 GB, 452 s
  - SHA256: `1bad0ee80a73e6dea1df2ef52505157584e104690c54267a46df15001fa91f8a`
  - GT: `gt_odom.txt`, 4520 poses at ~10 Hz, TUM format
  - GT SHA256: `d32c2ea794409e3787b341953b4a0fda3a1f27b5af598f79ba392e38897de2f4`
- 4DRadarSLAM: `dd2ee8c2378630a84361f37331ba14665b203c14` (upstream unmodified, plus 1 build-fix patch below)
- Launch: `radar_graph_slam.launch` (author's default; RViz node stripped, no other changes)
- Registration: FAST_APDGICP
- Loop closure: OFF (author's default)
- Frontend ego-vel init: OFF (author's default)
- Dynamic object removal: OFF (author's default)
- Playback: nominal `--rate=3`, effective ~1x (2-vCPU droplet throttled)

## Build-fix patch applied (upstream CMakeLists bug)
The upstream `catkin_make` succeeds but the scan_matching_odometry nodelet fails
to load at runtime because `keyframe.cpp` and g2o libs are only linked into
`radar_graph_slam_nodelet`, not `scan_matching_odometry_nodelet`. Fix in
`4DRadarSLAM/CMakeLists.txt`:

Add to `add_library(scan_matching_odometry_nodelet ...)`:
```
  src/radar_graph_slam/keyframe.cpp
```
Add to its `target_link_libraries(...)`:
```
  ${G2O_TYPES_DATA} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SLAM3D_ADDONS}
```
After patch, all 3 nodelets dlopen cleanly.

## Outputs (kept in results dir on droplet, small files copied here)
- `stamped_pose_graph_estimate.txt` — 30 keyframe poses, TUM format
- `map.pcd` — 1.3 MB radar point-cloud map (not committed)
- `graph/` — g2o pose graph + per-keyframe PCDs (not committed)
- `run.log` — full ROS launch log

## Evaluation (rpg_trajectory_evaluation, official, SE(3) alignment)

### ATE (Absolute Trajectory Error)
| Metric | Value |
|---|---:|
| Trajectory length | 245.85 m |
| Trans RMSE | 5.80 m |
| Trans mean | 4.31 m |
| Trans median | 3.11 m |
| Trans max | 14.78 m |
| Rot RMSE | 11.79° |
| n samples | 16 |

### KITTI-style Relative Error (RE)
| Sub-traj (m) | trans RMSE (m) | trans_perc RMSE | rot RMSE (°) | rot deg/m | n |
|---|---:|---:|---:|---:|---:|
| 24.6  | 8.72  | 35.5% | 14.4 | 0.58 | 8  |
| 49.2  | 6.35  | 12.9% | 12.5 | 0.25 | 13 |
| 73.8  | 13.39 | 18.2% | 17.5 | 0.24 | 12 |
| 98.3  | 16.33 | 16.6% | 24.4 | 0.25 | 11 |
| 122.9 | 20.92 | 17.0% | 29.1 | 0.24 | 9  |

### evo_ape cross-check (SE(3), 30 pose pairs, t_max_diff=0.15s)
| Metric | Value |
|---|---:|
| Trans RMSE | 7.32 m |
| Trans mean | 5.89 m |

(Numbers differ from RPG because evo matches 30 pairs on raw timestamps
whereas RPG resamples and matched 16 for ATE.)

## Comparison with paper
Paper reports ~2–4% rel trans err on `cp`. Ours is 13–18% — 3–4× worse.
Likely reasons, in order of suspicion:
1. Loop closure OFF (author top-level launch default), paper may use ON
2. Ego-vel init OFF, paper may use ON
3. Very sparse trajectory (30 kf / 246 m = one kf per 8 m) — long gaps between constraints
4. Real playback ~1x rather than intended 3x — should help, not hurt, but timing interactions unclear

## Comparison with evo sanity check
| Method | ATE trans RMSE |
|---|---:|
| RPG (official) | 5.80 m |
| evo_ape | 7.32 m |
Same order of magnitude, different because of different matching (RPG resamples).

## Milestones
- M1 local git repo skeleton    DONE
- M2 Ubuntu build of original   DONE
- M3 NTU4DRadLM baseline run    DONE (this file)
