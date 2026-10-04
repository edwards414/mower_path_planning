#!/bin/sh
# udhcpc dispatcher for wwan0 (run by mower-lte.sh, not the stock
# /etc/udhcpc/default.script): same address/DNS handling, but the default
# route is added at MOWER_LTE_METRIC so a wired or Wi-Fi uplink stays preferred.
METRIC=${MOWER_LTE_METRIC:-200}

log() { logger -t "mower-lte-dhcp" -p daemon."$1" "$interface: $2"; }

case $1 in
    bound|renew)
        busybox ifconfig "$interface" ${mtu:+mtu $mtu} "$ip" netmask "$subnet" ${broadcast:+broadcast $broadcast}
        busybox ip -4 route flush exact 0.0.0.0/0 dev "$interface"
        router="${router%% *}"
        if [ -n "$router" ]; then
            [ ".$subnet" = .255.255.255.255 ] && onlink=onlink || onlink=
            busybox ip -4 route add default via "$router" dev "$interface" metric "$METRIC" $onlink
        fi
        R=""
        for i in $dns; do R="$R
nameserver $i"; done
        [ -x /sbin/resolvconf ] && echo "$R" | resolvconf -a "$interface.udhcpc"
        log info "$1: IP=$ip/$subnet router=$router metric=$METRIC dns=\"$dns\" lease=$lease"
        ;;
    deconfig)
        busybox ip link set "$interface" up
        busybox ip -4 addr flush dev "$interface"
        busybox ip -4 route flush dev "$interface"
        [ -x /sbin/resolvconf ] && resolvconf -d "$interface.udhcpc"
        log notice "deconfigured"
        ;;
    leasefail|nak)
        log err "configuration failed: $1: $message"
        ;;
esac
