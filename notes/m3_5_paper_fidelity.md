# M3.5 — paper-fidelity baseline runs on `cp` (2026-09-30)

Verdict: **baseline frozen at ~5m ATE / ~12% RE49**, systematic 2× gap vs paper across all three tested configurations. Paper's numbers are not reachable from the released code + documented launch args alone.

## Motivation
M3 numbers (5.80m ATE) were on the default launch, which has loop closure OFF, dynamic-object removal OFF, ego-vel init OFF, and backend keyframe threshold 2m. Paper's `cp` results are: APDGICP 3.56% / 2.61m ATE, APDGICP+LC 3.02% / 2.35m ATE, with 437 keyframes. M3.5 tests whether the paper's numbers are reachable by adjusting these switches.

## Three controlled runs
All at nominal `--rate=3` (effective ~1x on 2 vCPU), 4DRadarSLAM commit `dd2ee8c` + CMakeLists fix.

- **Run A** `rate3_A_frontend_paper_config`: default + `keyframe_delta_trans_back_end:=0.5` (paper kf density). Records `/odom` (front-end APDGICP) and backend graph.
- **Run B** `rate3_B_dyn_and_egovel`: Run A + `enable_dynamic_object_removal:=true` + `enable_frontend_ego_vel:=true`.
- **Run C** `rate3_C_paper_kf_plus_loopclosure`: Run A + `enable_loop_closure:=true`.

## Results (RPG official, SE(3) alignment)

| Config | ATE (m) | RE49 (%) | RE49 (deg/m) | n | Notes |
|---|---:|---:|---:|---:|---|
| M3 default (backend only) | 5.80 | 12.93 | 0.2544 | 13 | 30 keyframes |
| A: frontend /odom | 5.15 | 11.44 | 0.3360 | 680 | 2062 poses |
| A: backend graph | 4.86 | 12.09 | 0.3515 | 45 | 142 keyframes |
| B: frontend /odom | **29.00** | **29.66** | **0.9105** | 892 | **broken** |
| C: frontend /odom | 5.20 | 13.08 | 0.4465 | 714 | 2048 poses |
| C: backend (LC on) | 5.32 | 15.14 | 0.2313 | 37 | 140 keyframes |
| **Paper APDGICP** | **2.61** | **3.56** | **0.0369** | – | ITSC 2023 |
| **Paper APDGICP+LC** | **2.35** | **3.02** | **0.0448** | – | ITSC 2023 |

## Findings
1. **All non-broken configs cluster at ~5m ATE / ~11-15% RE49.** Within noise of each other. Changing keyframe density, adding loop closure — no meaningful impact.
2. **~2× gap vs paper in translation, ~10× gap in rotation.** The rotation error gap (0.34 deg/m vs paper 0.037) is systematic, not tuning-level.
3. **Loop closure did not help on `cp`.** Only 37 backend samples matched for evaluation — suggests default LC thresholds don't fire on this sequence's geometry.
4. **`enable_dynamic_object_removal` + `enable_frontend_ego_vel` is a broken combination.** Turning both ON pushed ATE from 5m → 29m and RE49 from 11% → 30%. Documentation gotcha: even though the paper describes both mechanisms as helpful, enabling them together via the launch args in the released code degrades performance dramatically. Recommend leaving both OFF pending further investigation.

## Interpretation of the gap
The 10× rotation error gap suggests the paper's numbers use IMU-derived orientation somewhere that the released default launch does not activate. The launch has `enable_imu_fusion:=false` hard-coded ("bad effect, not used"). Yet without something providing orientation prior, front-end APDGICP alone accumulates rotation drift consistent with what we see.

The paper may:
- report best-of-N runs (unlikely to be published without disclosure)
- use per-sequence hand-tuned parameters not in the launch defaults
- use a different `cp` bag than what NTU4DRadLM currently publishes
- use IMU orientation fusion in a non-default code path

## Trustworthy baseline going forward
Use **Run A (`keyframe_delta_trans_back_end:=0.5`, everything else default)** as the reference:
- ATE trans RMSE: **5.15 m** (front-end) / **4.86 m** (back-end)
- RE49 trans_perc: **11.44%** (front-end) / **12.09%** (back-end)
- Reproducible from the same droplet/container/patch/commands documented in this note and `m3_cp_baseline.md`.

## Data preserved
Per run: `odom.tum`, `stamped_pose_graph_estimate.txt`, `saved_results/` (RPG stats yamls).
Not committed: bags, maps, graphs (too big).
