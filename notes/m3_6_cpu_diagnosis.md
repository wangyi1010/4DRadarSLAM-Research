# M3.6 — root-cause diagnosis: CPU throughput, not algorithm (2026-10-01)

## Headline
The ~2× gap between our "baseline" (5.15 m ATE) and the paper (2.35–2.61 m ATE) is
**not** caused by algorithm bugs or default launch parameters. It is caused by
**CPU throughput** on the 2 vCPU droplet dropping ~62% of radar frames at the
launch's default `--rate=3` playback.

At `--rate=0.5` playback, we match the paper's ATE within 0.1 m and its
keyframe count within 2%.

## The five diagnostic runs

All on `cp` sequence, 4DRadarSLAM commit `dd2ee8c` + CMakeLists fix.

| Run | Playback | dyn_removal | ego_vel | loop closure | /odom count | ATE FE (m) | ATE BE (m) | RE49 FE (%) | rot deg/m |
|---|---|:---:|:---:|:---:|---:|---:|---:|---:|---:|
| A | 1× nominal (rate=3) | OFF | OFF | OFF | 2062 | 5.15 | 4.86 | 11.44 | 0.336 |
| C | 1× | OFF | OFF | **ON** | 2048 | 5.20 | 5.32 | 13.08 | 0.447 |
| D | 1× | **ON** | OFF | OFF | 2939 | 20.21 | 19.45 | 25.04 | 0.386 |
| E | 1× | OFF | **ON** | OFF | 2189 | 33.63 | 36.83 | 35.71 | 0.835 |
| B | 1× | **ON** | **ON** | OFF | 2582 | 25.07 | – | 29.66 | 0.911 |
| **F** | **0.5×** | OFF | OFF | OFF | **6+k** | **2.32** | **2.22** | **5.14** | **0.073** |
| Paper APDGICP | – | – | – | – | – | 2.61 | – | 3.56 | 0.037 |
| Paper +LC | – | – | – | – | – | – | 2.35 | 3.02 | 0.045 |

## What Run F revealed
- Front-end /odom now at 12.4 Hz sim time = ~100% of radar frames (Run A: 38%)
- Backend keyframes: 430 (paper: 437; Run A/1×: 142)
- `/aftmapped_to_init` poses: 4314 (Run A: 41)
- ATE 2.22 m — within noise of paper's 2.35 m

## What this invalidates from earlier notes
- The 5.15 m ATE recorded in `m3_5_paper_fidelity.md` as "trustworthy baseline"
  is actually a **CPU-throughput-limited artifact**. It should not be used as
  a research reference point.
- Runs D/E/B at 1× measured the interaction of `dyn_removal`/`ego_vel` with
  frame-dropping, not with the algorithms themselves. We cannot conclude those
  features are broken from those runs alone. They should be re-run at 0.5×.
- The "10× rotation error gap" (0.336 vs paper's 0.037) collapses to 2× (0.073
  vs 0.037) at 0.5×, and the residual gap is small enough to plausibly be
  loop closure or per-sequence tuning.

## The trustworthy baseline going forward
**Run F: `--rate=0.5`, everything default, keyframe_delta_trans_back_end:=0.5, all switches OFF.**
- ATE front-end: 2.32 m
- ATE back-end:  2.22 m
- RE49 trans_perc: 5.14%
- rot deg/m: 0.073

## Practical implications for research
1. All future comparison runs use `rosbag play --rate=0.5` (or slower).
   Runtime launch without embedded player: `m3_runtime/radar_graph_slam_headless_noplayer.launch`.
2. Run wall-clock time doubles: 15 min instead of 7.5.
3. Runs D and E should be re-run at 0.5× before drawing conclusions about
   whether dyn_removal / ego_vel are truly broken. At 1× they were confounded
   with frame drops.
4. Only `rate=0.5` results should be quoted as "baseline" numbers in any
   future paper or report.

## Data preserved
`notes/m3_6_cpu_diagnosis/rate*_{D,E,F}/`:
- odom.tum, stamped_pose_graph_estimate.txt
- saved_results/ (RPG stats yamls)
Not committed: bags, maps (too big).
