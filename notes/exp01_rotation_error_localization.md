# exp-01 — Localizing the rotation error: front-end vs back-end (2026-10-01)

First post-reproduction diagnostic. Question: where does the residual ~2x
rotation/relative-error gap (vs paper) originate — scan matching (front-end) or
graph optimization (back-end)?

## Method
Reuse the M3.7 runs at 0.5x. For B' (dyn+ego, no LC) and Full (dyn+ego+LC),
evaluate the front-end `/odom` trajectory and the back-end optimized pose graph
separately against GT, across all RPG sub-trajectory lengths.

## Data

### B' (dyn+ego, no loop closure)
| Stage | ATE (m) | RE49 trans% | RE49 rot deg/m |
|---|---:|---:|---:|
| FE /odom | 1.648 | 4.63 | 0.0828 |
| BE graph | 1.672 | 4.80 | 0.0849 |

### Full (dyn+ego+LC)
| Stage | ATE (m) | RE49 trans% | RE49 rot deg/m |
|---|---:|---:|---:|
| FE /odom | 1.797 | 4.63 | 0.0851 |
| BE graph | **0.892** | 5.02 | 0.0872 |

### Rotation error vs sub-trajectory length (Full BE)
| subtraj (m) | rot deg/m |
|---:|---:|
| 24.6 | 0.1182 |
| 49.2 | 0.0872 |
| 73.8 | 0.0697 |
| 98.3 | 0.0598 |
| 122.9 | 0.0488 |

## Findings

1. **Rotation error is born in the front-end.** FE /odom already carries the
   full rotation error (~0.085 deg/m at 49 m). The back-end does not reduce it
   (0.0851 -> 0.0872, slightly worse). Scan matching owns this error, not the
   graph.

2. **Loop closure corrects global translation, not local rotation.** Full config
   ATE halves (1.80 -> 0.89 m) via LC, while RE49 rotation is essentially
   unchanged. LC fixes global geometry without touching short-window motion
   estimation.

3. **Error signature = high-frequency rotational jitter, not drift.** Rotation
   deg/m DECREASES with sub-trajectory length (0.118 at 24.6 m -> 0.049 at
   122.9 m). Per-scan rotational noise averaging out over longer windows. A
   systematic rotational bias would instead be roughly constant in deg/m.

## Conclusion
The residual ~2x rotation gap vs the paper is a **front-end APDGICP
rotational-precision problem**. Back-end graph optimization and loop closure
cannot close it (confirmed empirically). Any improvement must target the
scan-matching / local motion estimation stage.

## Candidate research directions (front-end only)
1. **IMU orientation fusion.** Launch hard-codes `enable_imu_fusion:=false`
   ("bad effect, not used"). The /vectornav/imu and /livox/imu streams are
   available at ~200 Hz. A properly weighted orientation prior could damp the
   per-scan rotational jitter. This is the most direct lever.
2. **Rotational regularization / covariance shaping in APDGICP.** The adaptive
   probability distribution weights points by range/azimuth/elevation variance;
   rotation is constrained indirectly. Tightening angular constraints or adding
   an explicit rotational term could help.
3. **Multi-frame / scan-to-submap rotational smoothing** at the front end
   (max_submap_frames currently 5, scan_to_map off).

## Next step (before coding)
Confirm direction 1 is viable: check whether the IMU orientation is consistent
with radar-derived rotation on a few frames, and whether enable_imu_fusion, when
turned on at 0.5x (not 1x), helps or hurts rotation specifically. That is the
first exp/* branch experiment.

## Milestone
Reproduction phase CLOSED. First research problem identified and localized.
