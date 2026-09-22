# mower_localize_core

The ROS-free core of Phase C of [`docs/ROS_FREE_PLAN.md`](../../../../docs/ROS_FREE_PLAN.md):
a Rust port of the parts of **robot_localization (Jazzy, 3.8.3)** this mower
actually runs, as configured by
[`mower_nav2/config/dual_ekf_navsat_params.yaml`](../../../mower_nav2/config/dual_ekf_navsat_params.yaml).

Pure core, in the style of `mower_battery/src/estimator.rs`: no r2r, no ROS, no
I/O, no clock — every entry point takes an explicit nanosecond timestamp, so the
same code runs live, in replay and under test. It builds and tests with a plain
`cargo test` on any host.

Upstream source: <https://github.com/cra-ros-pkg/robot_localization>, branch
`jazzy-devel`, commit `3efa714` ("Add ToLLArray service", #953), which is the
3.8.3 that `ros-jazzy-robot-localization` ships in the runtime image.

| Rust | Upstream |
|---|---|
| `src/ekf.rs` | `src/ekf.cpp` + `src/filter_base.cpp` |
| `src/filter_common.rs` | `include/robot_localization/filter_common.hpp`, plus the dense linear algebra Eigen provided |
| `src/measurement.rs` | `include/robot_localization/measurement.hpp` |
| `src/prepare.rs` | the used half of `src/ros_filter.cpp` |
| `src/navsat.rs` | `src/navsat_transform.cpp` |
| `src/utm.rs` | GeographicLib 2.3 `TransverseMercator.cpp`, `UTMUPS.cpp`, `Math.cpp` (via `navsat_conversions.hpp`) |
| `src/tf.rs` | the `tf2::LinearMath` operations those files use, and `angles::normalize_angle` |
| `src/msgs.rs` | the ROS messages, as plain structs |
| `src/config.rs` | `dual_ekf_navsat_params.yaml` itself |

## What the yaml turns on, and is therefore ported

**Both EKF instances** (`ekf_filter_node_odom`, world frame `odom`;
`ekf_filter_node_map`, world frame `map`): 20 Hz, `two_d_mode: true`,
`publish_tf: true`, `base_link_frame: base_footprint`, `use_control: false`,
their two different `process_noise_covariance` diagonals.

* the 15-state predict, with the same transfer function and the same analytic
  Jacobian, the control term (`prepareControl` /
  `computeControlAcceleration`, on the predict path even though
  `use_control` is false), state angle wrapping;
* the correct step: the update-index selection (including the NaN/Inf
  exclusion), the negative-covariance `fabs` and the 1e-9 variance floor, the
  Kalman gain via a partial-pivot LU inverse (what `Eigen::MatrixXd::inverse()`
  dispatches to), innovation angle wrapping, the Mahalanobis gate, the Joseph
  form covariance update;
* `processMeasurement`: first-measurement initialisation, predict only when the
  delta is positive (so out-of-order data corrects without predicting), and the
  `delta >= 0` rule for advancing `last_measurement_time_`;
* `integrateMeasurements`: the time-ordered queue, the sensor timeout, and
  `predict_to_current_time`;
* the preprocessing for the enabled sensors — `odom0` (and `odom1` on the map
  instance) and `imu0`: `odometryCallback`/`imuCallback` with the per-portion
  update vectors `loadParams` derives, `preparePose` (including the IMU
  mounting-yaw special case), `prepareTwist` (including the sensor-offset cross
  product with the state's angular velocity), `prepareAcceleration` (including
  gravity removal and the angular-acceleration term), the covariance rotations,
  `copyCovariance`, `forceTwoD`, the stale-message rejection, and
  `differentiateMeasurements`;
* `getFilteredOdometryMessage` and the `map -> odom` composition
  `periodicUpdate` broadcasts.

**`navsat_transform`**: `zero_altitude: true`, `publish_filtered_gps: true`,
`use_odometry_yaw: false`, `wait_for_datum: false`,
`magnetic_declination_radians: 0.0`, `yaw_offset: 0.0`,
`broadcast_cartesian_transform: true`.

* the datum (first good fix), LL <-> UTM through the same GeographicLib series,
  the meridian convergence, the yaw offset and magnetic declination, the
  `cartesian_world_transform` and its inverse, the GPS-antenna offset removal on
  both sides (`getRobotOriginCartesianPose`, `getRobotOriginWorldPose`),
  `prepareGpsOdometry` (what `/odometry/gps` carries) and `prepareFilteredGps`
  (`/gps/filtered`), plus the `toLL`/`fromLL` maths as `map_to_ll`/`from_ll`.
* `set_datum` and the manual-datum path are ported (they are what a future
  saved-site datum would use) but are off in this yaml.

## What is NOT ported, and why

Nothing below is reachable with this robot's configuration.

**Filters and estimators**

* `ukf.cpp` / `ukf_node.cpp` — the launch file runs `ekf_node`.
* `robot_localization_estimator.cpp`, `ros_robot_localization_listener.cpp` —
  a separate library for querying past states; nothing on this robot uses it.
* `use_dynamic_process_noise_covariance` — the scaling *is* ported (it is three
  lines in the predict path) but stays off; the yaml never sets it.

**RosFilter features the yaml leaves at their defaults**

* `smooth_lagged_data` / `history_length` — the whole lagged-data machinery
  (`revertTo`, `saveFilterState`, `clearExpiredHistory`, the measurement and
  filter-state history deques). Default false, not set. Out-of-order
  measurements are therefore handled the way upstream handles them without
  smoothing: correct without predicting.
* `use_control` — the control subscriptions (`/cmd_vel`, stamped or not) and
  `control_config` / acceleration and deceleration limits. `use_control: false`,
  so `prepareControl` zeroes the control acceleration every predict; the
  arithmetic is still ported so turning it on later is a config change.
* `print_diagnostics` / `debug` — `addDiagnostic`, `aggregateDiagnostics`, the
  frequency diagnostic, the covariance sanity warnings inside `copyCovariance`
  and every `FB_DEBUG`/`RF_DEBUG` line. Both false in the yaml (the diagnostics
  publisher was measured at ~1 % of a core per filter, see the plan).
* `publish_acceleration` — `/accel/filtered`, `getFilteredAccelMessage` and
  `angular_acceleration_cov_`. Default false. The angular *acceleration* itself
  is ported because `prepareAcceleration` consumes it.
* `poseN` / `twistN` / `accelN` inputs — the yaml only declares `odomN` and
  `imuN`. `preparePose`/`prepareTwist`/`prepareAcceleration` are ported in full,
  so adding such an input is a caller-side change.
* `differential` and `relative` — both false for every sensor here. The code
  paths are ported (they share `preparePose`), but they are not covered by the
  oracle vectors, so treat them as untested.
* `reset_on_time_jump` — its body is commented out upstream.
* `permit_corrected_publication`, `predict_to_current_time`,
  `disabled_at_startup`, `toggled_on_`, the `set_pose` / `reset` /
  `toggle_filter_processing` / `enable`/`disable` services, `initial_state`,
  `dynamic_process_noise_covariance`, `tf_time_offset`, `transform_timeout`,
  `queue sizes`, QoS overrides — publication policy, service plumbing or
  scheduling, none of it arithmetic.
* `validateDelta` — a no-op upstream (the body is commented out).

**tf**

`tf2_ros::Buffer` is replaced by [`TransformTree`](src/prepare.rs), a map of
fixed transforms, because every lookup this code path makes is between rigidly
attached frames (`base_footprint` -> `imu_link`, `base_footprint` -> `gps_link`)
or resolves to the identity (`target == source`). The time-tolerant second
attempt inside `lookupTransformSafe` therefore collapses into the same answer.
Publishing transforms is left to the caller; `map_to_odom()` returns the value
`periodicUpdate` would broadcast.

**navsat_transform**

* `use_local_cartesian` — `GeographicLib::LocalCartesian`/`Geocentric`. False in
  the yaml; the robot works in UTM.
* the polar UPS branch of `UTMUPS`, and MGRS zone strings (`LLtoUTM`'s zone
  string, the `setUTMZone` service, `MGRS::Forward`/`Reverse`).
  `utmups_forward` returns `UtmError::PolarRegionUnsupported` above 84 deg N or
  below 80 deg S.
* `TransverseMercatorExact` — `UTMUPS` uses the 6th-order Krüger series, which
  is what is ported.
* the services (`toLL`, `fromLL`, `toLLArray`, `fromLLArray`, `setDatum`,
  `setUTMZone`) and the transform broadcast: the maths behind them is ported,
  the ROS surface is not.
* `delay` (a startup sleep) and `frequency` (a timer period).

**Two deliberate small differences**

1. Upstream's measurement queue is a `std::priority_queue`, which leaves the
   order of equal timestamps unspecified; this port breaks ties in insertion
   order. The oracle run (which contains IMU messages that enqueue three
   measurements at one stamp, plus injected duplicate stamps) matches to 1e-15,
   so the two agree in practice.
2. Upstream keys `last_message_times_` per portion (`imu_pose`, `imu_twist`,
   `imu_acceleration`); this port keys it per sensor. All portions of a message
   share its stamp, so the stale-message decision is the same.

## Tests

The vectors in `tests/data/` are generated by `tests/oracle/oracle.cpp`, which
links the installed `librl_lib.so` and GeographicLib inside the
`mower-jazzy-test:tools` image and runs the real implementation on a
deterministic synthetic sequence (odom twists at 25 Hz, IMU at 10 Hz, GPS at
4 Hz, with out-of-sequence and duplicate timestamps injected):

```sh
docker run -d --name rf-c -e ROS_DOMAIN_ID=84 \
    -v "$PWD":/repo mower-jazzy-test:tools sleep infinity
docker exec rf-c bash /repo/src/mower_rs/crates/mower_localize_core/tests/oracle/run_oracle.sh
cargo test -p mower_localize_core
```

| test | vectors | tolerance | measured worst case |
|---|---|---|---|
| `ekf_core` | 137 measurements | 1e-9 (relative for the covariance) | 2.2e-16 state, 1.1e-16 covariance |
| `ros_filter` (odom instance) | 137 messages, 200 prepared measurements | 1e-9 | 5.2e-16 measurement, 1.2e-15 state |
| `ros_filter` (map instance) | 137 messages, 134 prepared measurements | 1e-9 | 5.2e-16 measurement, 1.3e-15 state |
| `navsat` UTM | 14 points forward and back | 1e-6 m | 9.3e-10 m forward, 7.9e-10 m reverse |
| `navsat` transform | 40 cycles of `/odometry/gps` and `/gps/filtered` | 1e-6 m | 1.6e-9 m position, 3.2e-9 m lat/lon |
