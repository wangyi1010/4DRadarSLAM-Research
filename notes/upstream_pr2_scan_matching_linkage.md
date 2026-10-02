# Upstream PR #2 (DRAFT, not yet opened) — scan_matching nodelet linkage

Branch `upstream-fix/scan-matching-linkage` (off upstream/main), pushed to fork
`yi-wang-ai/4DRadarSLAM`. Diff: CMakeLists.txt only, +6/-0. Independent of
PR #18 (the pointer-init fix); this is the build prerequisite that had to be
worked around just to exercise the nodelet.

## Title
Fix missing KeyFrame and g2o linkage for scan_matching_odometry_nodelet

## Before (clean build of upstream main @ dd2ee8c)
Library builds, but has one unresolved symbol (so the nodelet fails to dlopen):
`radar_graph_slam::KeyFrame::KeyFrame(unsigned long, ros::Time const&,
Eigen::Transform<double,3,1,0> const&, double,
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> const> const&)`.
Runtime: "Failed to load nodelet .../ScanMatchingOdometryNodelet ... undefined symbol".

## Fix
Add `src/radar_graph_slam/keyframe.cpp` to the target sources, and the g2o
libraries already used by the backend target (KeyFrame uses g2o::VertexSE3):
G2O_TYPES_DATA, G2O_CORE_LIBRARY, G2O_STUFF_LIBRARY, G2O_TYPES_SLAM3D,
G2O_TYPES_SLAM3D_ADDONS.

## After (clean rebuild from scratch)
- Build exit 0; `ldd -r` reports 0 undefined symbols (was 1).
- Nodelet loads in a full launch (no "Failed to load"); pipeline produces /odom ~12 Hz.

## Status: staged, NOT opened. Open with:
gh pr create --repo zhuge2333/4DRadarSLAM --base main \
  --head yi-wang-ai:upstream-fix/scan-matching-linkage \
  --title "Fix missing KeyFrame and g2o linkage for scan_matching_odometry_nodelet" --body-file <body>
