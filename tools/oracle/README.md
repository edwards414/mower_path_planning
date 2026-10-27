# Oracle image

The one ROS container the ROS removal keeps (E9-05). The Rust ports
(`mower_nav_core`, `mower_localize_core`, `mower_base_core`) are tested against
JSON vectors the real implementations produced. This image holds those
implementations at the versions the robot image ships, so the vectors can be
regenerated at any time, bit for bit where it matters.

| | version | why that one |
|---|---|---|
| base | `ros:jazzy-ros-base@sha256:103b6c41…`, libc6 2.39-0ubuntu8.9 | the robot image's builder base; same libc (libm) as the robot's `ros:jazzy-ros-core` |
| nav2 (10 packages) | 1.3.13 | `deploy/image/apt/runtime-apt.lock` |
| robot_localization | 3.8.3 | same |
| rmw_cyclonedds_cpp | 2.2.4 | same |
| twist_mux, robot_state_publisher, xacro | jazzy, as the robot | E4's oracles |
| rustc | 1.99.1 | the shipped `mower_rsd`'s, `src/mower_rs/rust-toolchain.toml` |

Every apt package is pinned by `oracle-apt.lock`: the Dockerfile installs those
versions (the ROS ones from snapshots.ros.org, the snapshot of the robot's sync)
and fails if dpkg ends up with anything else. To change it, copy the list the
failed build prints. The ROS packages the robot also runs, and libc6, must keep
the robot's versions.

It is also the container CI's `rust-ros` job runs `cargo test` and the rclpy
black-box tools in, so neither that job nor the vector job runs apt.

## Use

```sh
docker buildx build -f tools/oracle/Dockerfile -t mower-oracle:jazzy .   # from the repository root
docker run --rm -v "$PWD":/repo mower-oracle:jazzy tools/oracle/regen.sh  # all; or: nav, localize, base
docker run --rm -v "$PWD":/repo mower-oracle:jazzy tools/oracle/check.sh  # what CI checks
```

`check.sh` runs `regen.sh` and then:
- **The deterministic vectors** must come out byte for byte as committed:
  nav_core's seven components, all of localize_core and all of base_core.
- **The two node scenarios** are compared where a run does not vary:
  `controller_server.json` and `navigator.json` record the real nodes on the
  wall clock. Two runs in the same image differ in about a thousand lines,
  and the wall scenario's times and plan / follow_path counts vary by
  several rounds on nav2 itself (36 plans in the committed recording, 30 in
  a CI run on 2026-10-30).
  - `compare_scenarios.py` holds the fresh recording to the committed one on
    every goal's acceptance, status, error code and cancel answers, and on
    where the robot ends (5 cm, 0.1 rad). Times, counts and recoveries are
    printed, not compared.
  - A real node starved of CPU aborts goals it otherwise reaches. A
    recording made while the Mac was loaded by other builds failed this way,
    so a recording that fails that is made once more before the check fails.
  - The port's tests (`controller_server_oracle`, `navigator_oracle`) then
    run against the fresh recording as a report only, with a warning on
    the run when they disagree. They gate against the committed recording,
    on every field, in build.yml's mower_rs tests. The fresh recording is
    then put back.

`regen.sh` runs:

- `src/mower_rs/crates/mower_nav_core/tests/oracle/run_oracle.sh`: seven
  component harnesses plus the controller_server and navigator node scenarios
  (about 3 min);
- `src/mower_rs/crates/mower_localize_core/tests/oracle/run_oracle.sh`;
- `src/mower_rs/crates/mower_base_core/tests/oracle/generate.sh`, which
  fetches ros2_controllers and control_toolbox at pinned commits.

## CI

`.github/workflows/oracle.yml` (arm64, like the robot) runs `regen.sh`
monthly, on demand, and when this directory changes on main. It fails if a
vector changed. Run by hand with `publish`, it also:

- pushes `ghcr.io/edwards414/mower-oracle:jazzy` and `:jazzy-<commit>`;
- prints the digest in the job summary;
- keeps a `docker save` of the image as an artifact for 90 days. Copy that
  to R2 to keep it longer: packages.ros.org keeps only its latest sync.

## Where the committed vectors come from

- nav_core's components, localize_core and protocol_oracle.json reproduce
  byte for byte in this image.
- base_core's diff_drive_oracle.json and replay_synthetic.json are this
  image's output since E9-05.
  - The earlier ones were made on a Mac (Apple clang 21, Apple's libm), which
    reproduces them byte for byte at the same upstream commits.
  - This image (Ubuntu clang 18.1.3, glibc 2.39, the robot's libm) differs in
    284 of 4801 and 108 of 3506 floats: at most 3.7e-15 absolute and 132 ulps,
    and 1 ulp in the replay. Structure, integers and the replay's tx bytes are
    identical.
  - With the same FMA contraction on both, the rest is the two libms. A
    contraction-off build (or g++, -std=c++17) differs in more lines.
  - The port's tests (1e-12) pass against both.

Published digest (pinned by build.yml's `rust-ros` job):
`ghcr.io/edwards414/mower-oracle@sha256:18c55b256d68d13de55dbb781dbfa3264c6bef962cc80e4ef48eaaf437382296`, from main 49606d1 (run
41074506255, 2026-10-30; tags `jazzy` and
`jazzy-49606d1ddfe5f670f0c92cad22eaf90249cbb25b`). Its `docker save` is the
run's artifact `mower-oracle-jazzy-49606d1ddfe5f670f0c92cad22eaf90249cbb25b`
(519 MB, kept until 2027-01-28); `zstd -d` it and `docker load` it to have the
image without GHCR.
