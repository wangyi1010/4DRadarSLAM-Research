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
