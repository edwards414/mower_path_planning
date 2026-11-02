#!/bin/sh
# Network profiles of the outdoor stress harness (PLAN.md "Stress profiles"),
# on eth0 of the container it runs in: the sim's (netem_sim, uplink) or the
# client's (downlink). DDS is on lo and never shaped. Development only.
#
#   netem.sh PROFILE up|down      PROFILE = good | typ | bad | blk | clear
#
# The profile's RTT is split evenly between the two directions, delay and
# jitter alike; its loss is split so that a round trip sees the profile's
# loss (per direction 1 - sqrt(1 - L)). up = tbf 3 Mbit (the LTE uplink),
# down = 10 Mbit. blk is 100 % loss in both directions; `clear` removes it.
# Every call prints the qdisc it leaves.
set -eu
profile=${1:?profile}
dir=${2:-up}
dev=eth0
case "$dir" in up) rate=3mbit ;; down) rate=10mbit ;; *) echo "up or down" >&2; exit 2 ;; esac
case "$profile" in
  good) delay=40ms; jitter=10ms; loss=0.501%; corr="" ;;
  typ)  delay=80ms; jitter=30ms; loss=1.511%; corr="25%" ;;
  bad)  delay=150ms; jitter=50ms; loss=5.132%; corr="" ;;
  blk)  delay=40ms; jitter=0ms; loss=100%; corr="" ;;
  clear) tc qdisc del dev $dev root 2>/dev/null || true; tc qdisc show dev $dev; exit 0 ;;
  *) echo "unknown profile $profile" >&2; exit 2 ;;
esac
tc qdisc del dev $dev root 2>/dev/null || true
if [ "$jitter" = 0ms ]; then
  tc qdisc add dev $dev root handle 1: netem delay $delay loss $loss
else
  tc qdisc add dev $dev root handle 1: netem delay $delay $jitter distribution normal loss $loss ${corr:+$corr}
fi
tc qdisc add dev $dev parent 1: handle 2: tbf rate $rate burst 32kbit latency 400ms
tc qdisc show dev $dev
