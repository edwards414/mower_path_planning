# The outdoor harness's network sidecar and app client (deploy/sim/outdoor/README.md):
# tc (netem, tbf) for shaping a container's eth0, and Python with websockets 14
# for the app-side tools (probe_app.py, ws_teleop_soak.py, e2-verify's ws_*).
# Development only, never on a robot.
FROM debian:bookworm
RUN apt-get update \
 && apt-get install -y --no-install-recommends iproute2 iputils-ping procps python3 python3-pip python3-numpy \
 && pip3 install --no-cache-dir --break-system-packages websockets==14.2 \
 && apt-get purge -y python3-pip && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*
