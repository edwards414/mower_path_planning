#!/bin/bash
# Keeps the LTE data session up (qmi_wwan, /dev/cdc-wdm0 + wwan0): Quectel
# EC25 or SIMCom SIM7600G-H, any modem the qmi_wwan driver binds.
#
# Dials the APN over QMI, then runs udhcpc with mower-lte-dhcp.sh so wwan0 gets
# its address, a default route at MOWER_LTE_METRIC (default 200: Ethernet /
# Wi-Fi from NetworkManager sit at 100, so LTE only carries traffic when they
# are gone) and its DNS servers via resolvconf. A watchdog polls the QMI packet
# service state; when the session drops or the modem re-enumerates (it does on
# a USB brownout) everything is torn down and dialed again. systemd restarts
# the script itself if it dies.
#
# Environment (from /opt/mower/.env): MOWER_LTE_APN (default internet),
# MOWER_LTE_METRIC (default 200), MOWER_LTE_QMI (default /dev/cdc-wdm0).
set -u

APN=${MOWER_LTE_APN:-internet}
METRIC=${MOWER_LTE_METRIC:-200}
QMI=${MOWER_LTE_QMI:-/dev/cdc-wdm0}
IFACE=${MOWER_LTE_IFACE:-wwan0}
DHCP_SCRIPT=${MOWER_LTE_DHCP_SCRIPT:-/opt/mower/host/mower-lte-dhcp.sh}
# A 4G session that drops is not noticed until the next poll, and the redial
# itself takes ~30 s (measured 2026-10-07: 17:29:52 down -> 17:30:25 new
# lease); 5 s keeps the dead time close to that floor.
POLL_S=5
RETRY_S=15

export MOWER_LTE_METRIC=$METRIC
udhcpc_pid=
cid=

log() { echo "$*"; }

modem_present() { [ -c "$QMI" ] && [ -d "/sys/class/net/$IFACE" ]; }

teardown() {
    if [ -n "$udhcpc_pid" ] && kill -0 "$udhcpc_pid" 2>/dev/null; then
        kill "$udhcpc_pid" 2>/dev/null
        wait "$udhcpc_pid" 2>/dev/null
    fi
    udhcpc_pid=
    if modem_present; then
        [ -n "$cid" ] && timeout 10 qmicli -d "$QMI" --client-cid="$cid" --wds-stop-network=disable-autoconnect >/dev/null 2>&1
        ip -4 addr flush dev "$IFACE" 2>/dev/null
        ip -4 route flush dev "$IFACE" 2>/dev/null
    fi
    resolvconf -d "$IFACE.udhcpc" 2>/dev/null
    cid=
}
trap 'teardown; exit 0' TERM INT

dial() {
    # qmi_wwan needs raw-IP framing for the EC25; the link must be down to change it.
    ip link set "$IFACE" down
    echo Y > "/sys/class/net/$IFACE/qmi/raw_ip" 2>/dev/null
    ip link set "$IFACE" up
    local out
    out=$(timeout 30 qmicli -d "$QMI" --wds-start-network="apn=$APN,ip-type=4" --client-no-release-cid 2>&1) || {
        log "start-network failed: $(echo "$out" | tr '\n' ' ')"
        return 1
    }
    cid=$(echo "$out" | sed -n "s/.*CID: '\([0-9]*\)'.*/\1/p")
    [ -n "$cid" ] || { log "no CID in: $(echo "$out" | tr '\n' ' ')"; return 1; }
    udhcpc -i "$IFACE" -f -s "$DHCP_SCRIPT" -t 6 -T 3 -A 10 -S &
    udhcpc_pid=$!
    log "dialed apn=$APN (wds cid $cid), udhcpc pid $udhcpc_pid"
}

connected() {
    modem_present || return 1
    kill -0 "$udhcpc_pid" 2>/dev/null || return 1
    timeout 10 qmicli -d "$QMI" --client-cid="$cid" --client-no-release-cid \
        --wds-get-packet-service-status 2>/dev/null | grep -q "'connected'"
}

while true; do
    until modem_present; do
        log "waiting for $QMI / $IFACE"
        sleep "$RETRY_S"
    done
    if ! dial; then
        teardown
        sleep "$RETRY_S"
        continue
    fi
    while sleep "$POLL_S"; do
        connected && continue
        log "session down (modem present: $(modem_present && echo yes || echo no)); redialing"
        break
    done
    teardown
    sleep 3
done
