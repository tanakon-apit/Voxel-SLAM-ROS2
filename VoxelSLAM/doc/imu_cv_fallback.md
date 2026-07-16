# Constant-Velocity Fallback for IMU Dropouts

## Problem

If the IMU stream has holes (e.g. logging loss of ~1.5 s), stock Voxel-SLAM
degrades in a hidden way: `sync_packages()` silently **drops every scan** whose
span is not covered by more than 4 IMU samples, and the first scan after the
hole is propagated from a ~1.5 s stale state. The registration usually still
"succeeds", absorbing a large pose error into the trajectory and local map.
The corrupted segment enters the pose graph with confident covariances, so the
next loop closure produces a large jump, and the rebuilt map is misaligned —
post-loop drift.

## Design

When a hole larger than `imu_gap_thresh` appears inside the current scan span,
`fill_imu_gaps()` (voxelslam.hpp) pads it with **synthetic IMU samples at the
nominal rate** that encode a constant-velocity motion model:

| quantity | synthetic value | effect after bias/gravity correction |
|---|---|---|
| angular velocity | `bg` (current gyro bias) | `ω = 0` → attitude `R` held constant |
| linear acceleration | `(ba − Rᵀ·g) / scale_gravity` | specific force cancels gravity → `a_world = 0`, so `v` constant, `p += v·dt` |

Because the padding consists of ordinary `sensor_msgs/Imu` messages (tagged
`frame_id = "cv_synth"`), **every downstream consumer works unchanged**:

- **EKF propagation** (`IMUEKF::motion_blur`) integrates them like real data,
  but pairs touching a `cv_synth` sample get their process noise inflated by
  `imu_cv_cov_scale` (default 100×), so the scan-to-map **LiDAR update
  dominates** the state correction during the gap — effectively LiDAR odometry
  with a CV prior.
- **Motion compensation (deskew)** uses the same propagated poses → linear
  interpolation across the scan, which is exactly the CV assumption.
- **Local-mapping BA preintegration** (`IMU_PRE::push_imu`) receives the padded
  stream, so the sliding-window IMU factors stay well-defined; the LiDAR plane
  factors constrain what the CV prior leaves open.
- The scan is **no longer dropped**: `sync_packages()` accepts under-covered
  scans whenever the fallback is enabled and the EKF is initialized.

## Switching states

```mermaid
stateDiagram-v2
    direction LR
    [*] --> INIT

    INIT : System / EKF initialization
    INIT : fallback inactive (no velocity or gravity estimate yet)

    IMU : IMU PROPAGATION (normal)
    IMU : real samples drive EKF prediction,
    IMU : deskew and preintegration

    CV : CONSTANT-VELOCITY FALLBACK
    CV : holes padded with cv_synth samples
    CV : ω = ω̂ from rate observer (or 0), a_world = 0
    CV : noise ×imu_cv_cov_scale
    CV : LiDAR registration dominates the update

    INIT --> IMU : EKF initialized (init_flag)
    IMU --> CV : hole > imu_gap_thresh inside scan span
    CV --> CV : gap continues (scan span fully or partly synthetic)
    CV --> IMU : scan span fully covered by real IMU again
    CV --> INIT : registration degenerates for degrade_bound scans (divergence reset, stock path)
```

Per-scan decision (inside the odometry loop, before `odom_ekf.process`):

```mermaid
flowchart TD
    A[scan + collected IMU samples] --> B{EKF initialized and\nimu_cv_fallback on?}
    B -- no --> C[stock behaviour:\nscan dropped if ≤ 4 samples]
    B -- yes --> D{hole > imu_gap_thresh between\nprev scan end / samples / scan end?}
    D -- no --> E[IMU propagation\nunchanged]
    D -- yes --> F[insert cv_synth samples\nat nominal period]
    F --> G[EKF propagate + deskew + preintegrate\nwith inflated noise on synthetic pairs]
    G --> H[scan-to-map EKF update\ncorrects the CV prediction]
```

The transition is logged on stdout:

```
IMU gap at scan t=...: constant-velocity fallback ON (N synthetic samples)
IMU stream recovered at t=...: back to IMU propagation
```

## Rate observer (`imu_cv_rate_observer`)

The pure CV model assumes **zero angular rate** during a gap — the EKF state
`[R, p, v, bg, ba, g]` contains no ω to hold. The optional rate observer
(`RateObserver`, voxelslam.hpp) removes that assumption:

- A 3-state EKF over the body angular velocity `ω`, constant-rate process
  model (`rate random walk q`), measured at scan rate from the
  registration-corrected orientation: `z = Log(R_prevᵀ · R_curr)/Δt`.
- It **runs on every scan** (with or without real IMU data), so at gap onset
  it already holds a smoothed estimate of the recent turn rate — no cold start.
- It is **consumed only by `fill_imu_gaps()`**: the synthetic gyro becomes
  `bg + ω̂` and the synthetic accel cancels gravity under the rotating attitude
  `R(t) = R₀·Exp(ω̂·t)`, so the padded motion is constant-rate + constant-velocity.
- **Health gating**: scans where registration failed or the odometry is
  degenerate drop the measurement base — a bad orientation never enters the
  finite difference. While the fallback itself is padding, the measurement
  noise is inflated 10× (R is then correlated with the observer's own
  prediction through the deskew).
- **Graceful decay**: the output is `ω̂ · var_cap/(var_cap + tr(P)/3)` — as the
  covariance grows (no healthy measurements, e.g. LiDAR lost too), the held
  rate fades smoothly back to zero, recovering the conservative CV model.

```mermaid
flowchart LR
    subgraph every scan
        REG[scan-to-map registration\ncorrected R] -->|healthy only| OBS[rate observer EKF\nz = Log dR / dt]
    end
    OBS -->|"ω̂ · confidence(P)"| GAP[fill_imu_gaps\nsynthetic gyro = bg + ω̂]
    GAP --> DESKEW[propagation + deskew\nwith held turn rate]
    DESKEW --> REG
```

## Parameters (`Odometry` section)

| parameter | default | meaning |
|---|---|---|
| `imu_cv_fallback` | `false` | enable the fallback (stock behaviour when off) |
| `imu_gap_thresh` | `0.05` s | hole size that triggers padding; also the upper bound for the nominal-period estimator. **Must stay below the scan period** (0.1 s at 10 Hz) — at or above it, a hole spanning most of one scan is never padded and the scan propagates with almost no samples |
| `imu_cv_cov_scale` | `100.0` | process-noise inflation for synthetic pairs |
| `imu_cv_rate_observer` | `false` | hold the observed turn rate through gaps instead of zero rotation |
| `rate_obs_q` | `0.01` | observer process noise [(rad/s)²/s]; higher tracks faster turns, lower smooths more |
| `rate_obs_r` | `0.002` | observer measurement noise [(rad/s)²] of `Log(dR)/Δt` |
| `rate_obs_var_cap` | `0.04` | variance [(rad/s)²] at which the held rate is half-trusted (decay toward zero) |

The nominal IMU period is estimated online (EMA over live inter-sample gaps
smaller than `imu_gap_thresh`), so no rate parameter is needed.

## Limitations

- Constant velocity is an assumption: during aggressive rotation inside a gap,
  deskew and the prior are wrong; the inflated noise lets registration correct
  the pose, but a long gap in feature-poor surroundings can still degenerate —
  in that case the stock divergence reset (new session + relocalization) takes
  over, which is the intended safety net. The rate observer mitigates the
  steady-turn case (rate held, not zeroed) but cannot see rate *changes* that
  happen inside a gap with no scans.
- Gyro/accel biases are held constant through the gap (they are unobservable
  without real IMU data).
- The BA preintegration factors treat synthetic samples with nominal IMU noise
  (inflation currently applies to the EKF only); the window's LiDAR factors
  dominate in practice.
