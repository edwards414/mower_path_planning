# zenoh-bridge-ros2dds

This folder contains an optional Zenoh setup for bridging ROS 2 traffic between
the robot container and a host machine running tools such as `rviz2`.

The setup follows the ROS 2 Jazzy and Eclipse Zenoh guidance for
`zenoh-bridge-ros2dds`:

- keep ROS 2 nodes on each machine local-only for DDS
- run one Zenoh bridge near the robot
- run one Zenoh bridge on the host that connects to the robot bridge

## Robot side

Rebuild the devcontainer image after adding `rmw_cyclonedds_cpp`, then start
the robot stack with the Zenoh overlay:

```bash
docker compose \
  -f .devcontainer/docker-compose.raspi.dev.yaml \
  -f .devcontainer/docker-compose.raspi.zenoh.yaml \
  up -d
```

The overlay does three things:

- switches ROS 2 in the container to `rmw_cyclonedds_cpp`
- sets `ROS_LOCALHOST_ONLY=1` to keep DDS local to the robot host
- starts `zenoh_bridge_ros2dds` listening on `tcp/0.0.0.0:7447`

## Host side

Install `zenoh-bridge-ros2dds` on the host, or run it with Docker. Then use:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///absolute/path/to/.devcontainer/zenoh/cyclonedds-localhost.xml

zenoh-bridge-ros2dds \
  -c /absolute/path/to/.devcontainer/zenoh/bridge-ros2dds-host.json5 \
  -e tcp/<robot-ip>:7447
```

After the host bridge is connected, start tools like:

```bash
rviz2
ros2 topic list
```

## Host RViz in `tiryoh/ros2-desktop-vnc`

This repo also includes a browser-based RViz host viewer built on
`tiryoh/ros2-desktop-vnc:${ROS_DISTRO}`.

Start it with:

```bash
cp .devcontainer/host-rviz.zenoh.env.example .devcontainer/host-rviz.zenoh.env
```

Edit `.devcontainer/host-rviz.zenoh.env`, then start it with:

```bash
docker compose \
  --env-file .devcontainer/host-rviz.zenoh.env \
  -f .devcontainer/docker-compose.host-rviz.zenoh.yaml \
  up -d
```

Then open:

```text
http://127.0.0.1:6081/
```

The stack starts:

- `rviz_host`: a VNC desktop with `rviz2` auto-launched
- `zenoh_bridge_host`: a host-side `zenoh-bridge-ros2dds` sharing the same
  localhost DDS network namespace as RViz

`rviz2` uses `config.rviz` from:

```text
/workspace/src/mower_bringup/config/config.rviz
```

The `.env` file values used by this stack are:

- `ROBOT_IP`: robot-side Zenoh bridge address
- `ROS_DISTRO`: the `tiryoh/ros2-desktop-vnc` tag to build
- `ROS_DOMAIN_ID`: ROS 2 domain for both host-side RViz and bridge

## Notes

- Keep `ROS_DOMAIN_ID` the same on both sides.
- Keep `ROS_LOCALHOST_ONLY=1` on both sides when using the bridge, otherwise
  you can get duplicate DDS traffic and loops.
- The bridge is tested primarily with `rmw_cyclonedds_cpp`.
