#!/usr/bin/env python3
"""Write <state_dir>/link_status.json every few seconds for the robot container.

The container (mower_mission telemetry_node) has no access to the LTE modem
or the host's network interfaces beyond what /proc exposes, so this host
service samples them and drops one small JSON file into the shared state dir::

    {"time": 1789621163,
     "lte": {"present": true, "rssi_dbm": -83, "csq": 15, "ber": 99,
             "registered": true, "operator": "Chunghwa", "tech": "LTE", "error": null},
     "wifi": [{"iface": "wlan0", "rssi_dbm": -61, "link": 49}],
     "ifaces": [{"name": "eth0", "up": true, "addr": "192.168.0.113"}, ...]}

Usage: mower-link-status.py [state_dir] [--once]
The AT port is /dev/lte_at (udev/99-mower.rules); missing modem = present:false.
"""

import json
import os
import re
import select
import subprocess
import sys
import termios
import time

AT_PORT = os.environ.get('MOWER_LTE_AT', '/dev/lte_at')
PERIOD_S = float(os.environ.get('MOWER_LINK_PERIOD_S', '5'))
SKIP_IFACES = ('lo', 'docker', 'veth', 'br-', 'dummy')


def configure_port(fd, baud=termios.B115200):
    """Raw 8N1, no echo, no flow control. A freshly enumerated modem tty
    comes up cooked at 9600 baud with CRTSCTS set, on which a non-blocking
    write fails with EAGAIN before the modem sees a byte (SIM7600G-H,
    2026-10-07)."""
    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, _ispeed, _ospeed, cc = attrs
    iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP
               | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON
               | termios.IXOFF | termios.IXANY)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG
               | termios.IEXTEN)
    cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
    cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
    cc = list(cc)
    cc[termios.VMIN], cc[termios.VTIME] = 0, 0
    termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, baud, baud, cc])


def at(fd, cmd, timeout=1.5):
    os.write(fd, (cmd + '\r\n').encode())
    out = b''
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        r, _, _ = select.select([fd], [], [], deadline - time.monotonic())
        if not r:
            break
        out += os.read(fd, 1024)
        if b'OK' in out or b'ERROR' in out:
            break
    return out.decode(errors='replace')


def parse_cpsi(text):
    """SIMCom (SIM7600) serving-cell report ->
    {'band', 'plmn', 'rsrp_dbm', 'rsrq_db', 'sinr_db'} or None.

    +CPSI: LTE,Online,466-92,0x639D,134534710,430,EUTRAN-BAND7,3050,5,5,-103,-998,-722,13
    After the band: EARFCN, DL/UL bandwidth, then RSRQ, RSRP and RSSI in
    0.1 dB(m) units and RSSNR in dB.
    """
    m = re.search(r'\+CPSI:\s*LTE,[^,]*,(\d+-\d+),[^,]*,[^,]*,[^,]*,([^,]*),'
                  r'-?\d+,-?\d+,-?\d+,(-?\d+),(-?\d+),(-?\d+),(-?\d+)', text)
    if not m:
        return None
    return {'band': m.group(2), 'plmn': m.group(1).replace('-', ''),
            'rsrq_db': round(int(m.group(3)) / 10.0, 1),
            'rsrp_dbm': round(int(m.group(4)) / 10.0, 1),
            'sinr_db': float(m.group(6))}


def lte():
    if not os.path.exists(AT_PORT):
        return {'present': False}
    info = {'present': True, 'rssi_dbm': None, 'csq': None, 'ber': None,
            'registered': None, 'operator': None, 'tech': None, 'error': None,
            'band': None, 'plmn': None, 'rsrp_dbm': None, 'sinr_db': None, 'rsrq_db': None}
    try:
        fd = os.open(AT_PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as e:
        info['error'] = str(e)
        return info
    try:
        configure_port(fd)
        termios.tcflush(fd, termios.TCIOFLUSH)
        m = re.search(r'\+CSQ:\s*(\d+),(\d+)', at(fd, 'AT+CSQ'))
        if m:
            csq, ber = int(m.group(1)), int(m.group(2))
            info['csq'], info['ber'] = csq, ber
            # 3GPP 27.007: 0 = -113 dBm, 31 = -51 dBm, 99 = unknown
            info['rssi_dbm'] = None if csq == 99 else -113 + 2 * csq
        m = re.search(r'\+CREG:\s*\d+,(\d+)', at(fd, 'AT+CREG?'))
        if m:
            info['registered'] = m.group(1) in ('1', '5')
        m = re.search(r'\+COPS:\s*\d+,\d+,"([^"]*)",?(\d+)?', at(fd, 'AT+COPS?'))
        if m:
            info['operator'] = m.group(1)
            info['tech'] = {'0': 'GSM', '2': 'UTRAN', '7': 'LTE'}.get(m.group(2) or '', m.group(2))
        # Quectel extras: the serving band and the LTE quality numbers CSQ
        # hides. +QNWINFO: "FDD LTE","46692","LTE BAND 7",3050
        m = re.search(r'\+QNWINFO:\s*"([^"]*)","(\d*)","([^"]*)",(\d+)', at(fd, 'AT+QNWINFO'))
        if m:
            info['band'] = m.group(3)
            info['plmn'] = m.group(2)
        # +QCSQ: "LTE",<rssi>,<rsrp>,<sinr>,<rsrq>; sinr is (value/5 - 20) dB
        m = re.search(r'\+QCSQ:\s*"LTE",(-?\d+),(-?\d+),(-?\d+),(-?\d+)', at(fd, 'AT+QCSQ'))
        if m:
            info['rsrp_dbm'] = int(m.group(2))
            info['sinr_db'] = round(int(m.group(3)) / 5.0 - 20.0, 1)
            info['rsrq_db'] = int(m.group(4))
        if info['rsrp_dbm'] is None:  # SIMCom: the same numbers from +CPSI?
            cell = parse_cpsi(at(fd, 'AT+CPSI?'))
            if cell:
                info.update(cell)
    except OSError as e:
        info['error'] = str(e)
    finally:
        os.close(fd)
    return info


def wifi():
    out = []
    try:
        with open('/proc/net/wireless') as f:
            for line in f.readlines()[2:]:
                parts = line.split()
                if len(parts) < 4:
                    continue
                out.append({'iface': parts[0].rstrip(':'),
                            'link': float(parts[2].rstrip('.')),
                            'rssi_dbm': float(parts[3].rstrip('.'))})
    except OSError:
        pass
    return out


def ifaces():
    out = []
    try:
        text = subprocess.run(['ip', '-j', '-4', 'addr'], capture_output=True, text=True, timeout=3).stdout
        for it in json.loads(text or '[]'):
            name = it.get('ifname', '')
            if name.startswith(SKIP_IFACES):
                continue
            addrs = [a.get('local') for a in it.get('addr_info', []) if a.get('family') == 'inet']
            out.append({'name': name, 'up': it.get('operstate') in ('UP', 'UNKNOWN'),
                        'addr': addrs[0] if addrs else None})
    except (OSError, ValueError, subprocess.SubprocessError):
        pass
    return out


def write(state_dir):
    doc = {'time': int(time.time()), 'lte': lte(), 'wifi': wifi(), 'ifaces': ifaces()}
    tmp = os.path.join(state_dir, 'link_status.json.tmp')
    with open(tmp, 'w') as f:
        json.dump(doc, f)
    os.replace(tmp, os.path.join(state_dir, 'link_status.json'))
    try:
        st = os.stat(state_dir)
        os.chown(os.path.join(state_dir, 'link_status.json'), st.st_uid, st.st_gid)
    except OSError:
        pass
    return doc


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    state_dir = args[0] if args else '/home/cat/.mower'
    if '--once' in sys.argv:
        print(json.dumps(write(state_dir), indent=1))
        return
    while True:
        try:
            write(state_dir)
        except Exception as e:  # keep the loop alive; the container just sees a stale file
            print(f'link-status: {e}', file=sys.stderr)
        time.sleep(PERIOD_S)


if __name__ == '__main__':
    main()
