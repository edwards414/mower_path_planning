#!/bin/bash
# Read-only measurement: 10 s top + loopback packet rate + DDS thread split + process table + recent log lines.
echo "== $(date) temp=$(cat /sys/class/thermal/thermal_zone0/temp) freq=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq) container=$(sudo -n docker ps --format '{{.Image}} {{.Status}}' --filter name=lawan)"
r1=$(awk "/^ *lo:/{print \$3}" /proc/net/dev)
top -b -n 2 -d 10 -w 200 | awk "/^top/{n++} n==2" | sed -n '1p;3p;7,45p' | awk 'NR<=2 || $9+0>=0.5'
r2=$(awk "/^ *lo:/{print \$3}" /proc/net/dev); echo "lo_rx_pkts_per_s $(( (r2-r1)/10 ))"
echo "== dds threads 5 s"; top -H -b -n 2 -d 5 -w 200 | awk "/^top/{n++} n==2" | grep -E "recv|tev|dq\." | awk "{s[\$NF]+=\$9} END{for(k in s) printf \"%s %.1f\n\", k, s[k]}"
echo "== procs"; sudo -n docker top mower-lawan_node-1 -o pid,etimes,comm | awk 'NR>1{print $3}' | sort | uniq -c | sort -rn | head -40
echo "== errors 5 min: $(sudo -n docker logs --since 5m mower-lawan_node-1 2>&1 | grep -c -iE '\[ERROR\]|error:')"
sudo -n docker logs --since 5m mower-lawan_node-1 2>&1 | grep -iE "Managed nodes are active|mower_rsd|component_container|\[ERROR\]" | grep -viE "imu|gps" | tail -8
