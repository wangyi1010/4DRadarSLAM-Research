# 4DRadarSLAM Baseline Reproduction

## Goal
Reproduce the original 4DRadarSLAM implementation before introducing any
algorithmic modifications.

## Baseline
- Upstream: zhuge2333/4DRadarSLAM (git remote `upstream`)
- Frozen at branch `baseline` / tag `baseline-original`
- License: GPL-3.0 (retained)

## M2 target environment (VERIFIED 2026-09-30)
Host: DigitalOcean droplet, Frankfurt (FRA1)
- Ubuntu 22.04 LTS, 2 vCPU, 3.8 GB RAM, 80 GB disk
- Added: 4 GB swap (required — 3.8 GB RAM is not enough to link g2o/PCL)
- Cost: $0.036/hr, ~$5 credit covers it; DESTROY when idle.

ROS environment: Docker `ros:noetic-perception` (Ubuntu 20.04 focal inside)
- Persistent container name: `radar`
- Workspace mounted at `/root/catkin_ws` (host <-> container)
- Build: `catkin_make -j2 -DCMAKE_BUILD_TYPE=Release`

## Dependencies (actual, corrects the upstream README)
### apt (inside container)
- ros-noetic-geodesy, ros-noetic-pcl-ros, ros-noetic-nmea-msgs, ros-noetic-libg2o
- ros-noetic-interactive-markers  <-- README omits
- ros-noetic-nodelet              <-- README omits
- ros-noetic-tf-conversions       <-- README omits
- ros-noetic-eigen-conversions    <-- README omits
- ros-noetic-rviz                 <-- README omits
- build-essential, cmake, libomp-dev, libeigen3-dev, python3-catkin-tools

### PPA
- Borglab GTSAM 4.0 (ppa:borglab/gtsam-release-4.0)
  → libgtsam-dev, libgtsam-unstable-dev (4.0.3)  <-- README omits

### Source (into `catkin_ws/src`)
- zhuge2333/4DRadarSLAM             (the project)
- zhuge2333/fast_apdgicp (--recursive)
- zhuge2333/barometer_bmp388
- koide3/ndt_omp                    <-- README omits

## Build result
`catkin_make -j2 Release` exits 0. Artifacts in `devel/lib`:
- 3 nodelets: preprocessing_nodelet, scan_matching_odometry_nodelet, radar_graph_slam_nodelet
- 2 executables: gt_adjust, gps_traj_align
- Scripts: bag_player.py, ford2bag.py, map2odom_publisher.py

## Run entry (M3, next)
`roslaunch radar_graph_slam radar_graph_slam.launch`
Play NTU4DRadLM bag via launch/rosbag_play_radar_*.launch

## Milestones
- M1  local git repo skeleton      DONE
- M2  Ubuntu build of original     DONE  (2026-09-30)
- M3  NTU4DRadLM baseline run      pending

## Pinned source versions (M2 build, 2026-09-30)
Recorded from the compiled droplet workspace — these are the exact SHAs `catkin_make` succeeded against.

| Package | Upstream | Commit |
|---|---|---|
| 4DRadarSLAM | zhuge2333/4DRadarSLAM | dd2ee8c2378630a84361f37331ba14665b203c14 |
| fast_apdgicp | zhuge2333/fast_apdgicp | 8d1e1188f2566ec19ae73b842f90c96df66dde74 |
| ndt_omp | koide3/ndt_omp | 5495fd9214945afcb4b35d5a1da385e405c52bf9 |
| barometer_bmp388 | zhuge2333/barometer_bmp388 | 769411c24242faf15cd111f79b515e905a4c1550 |

The `4DRadarSLAM` SHA matches this repo's `baseline` branch / `baseline-original` tag exactly.

## M3 baseline result (2026-09-30, cp sequence)
See notes/m3_cp_baseline.md. Summary:
- ATE trans RMSE: 5.80 m over 245.85 m path
- Rel trans err at 49.2 m sub-traj: 12.9%
- Vanilla config (loop closure OFF, ego-vel OFF); ~3-4x worse than paper.
- Build-fix patch to CMakeLists needed: keyframe.cpp + g2o libs into scan_matching_odometry_nodelet.
