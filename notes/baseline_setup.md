# 4DRadarSLAM Baseline Reproduction

## Goal
Reproduce the original 4DRadarSLAM implementation before introducing any
algorithmic modifications.

## Baseline
- Upstream repo: zhuge2333/4DRadarSLAM (git remote `upstream`)
- Frozen at branch `baseline` / tag `baseline-original`
- License: GPL-3.0 (retained; derivative work must comply)

## Target environment
- Ubuntu 18.04/20.04, ROS Melodic/Noetic
- Deps: Eigen3, OpenMP, PCL, g2o, author's fast_apdgicp
- macOS here is for Git / reading / editing only — build & run on Ubuntu.

## Dataset
- NTU4DRadLM (junzhang2016/NTU4DRadLM), stored under ~/Datasets/NTU4DRadLM
- Initial sequence: cp

## Run entry (Ubuntu, later)
- roslaunch radar_graph_slam radar_graph_slam.launch

## Metrics
- ATE, RE (relative error), runtime

## Milestones
- M1  local git repo skeleton      <- current
- M2  Ubuntu build of original code
- M3  NTU4DRadLM baseline run

## Status
Not yet reproduced. M1 skeleton set up on branch `dev`.
