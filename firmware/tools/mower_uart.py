#!/usr/bin/env python3
"""Bench test tool for the STM32 mower UART protocol (UART_OPEN_LOOP_PROTOCOL.md).

usage:
  mower_uart.py PORT monitor [SECONDS]
  mower_uart.py PORT wheel L R [SECONDS]        # permille -1000..1000
  mower_uart.py PORT lawer DUTY [SECONDS]       # permille 0..1000 (blade is single-direction)
  mower_uart.py PORT led MODE [R G B PERIOD_MS] # mode 0..5
  mower_uart.py PORT pid KP KI [KD] [--save]    # both wheels, RAM only unless --save
  mower_uart.py PORT info                        # ask for the 0x87 firmware info frame
  mower_uart.py PORT servo PULSE_US [HOLD_MS] [SECONDS]  # 500..2500, 0 = release; HOLD_MS 0 = hold forever

needs: pip install pyserial
"""
import struct
import sys
import time

import serial

SOF = b"\xA5\x5A"
VER = 0x01


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def build(ftype: int, seq: int, payload: bytes) -> bytes:
    body = bytes([VER, ftype, seq & 0xFF, len(payload)]) + payload
    return SOF + body + struct.pack("<H", crc16_ccitt_false(body))


class Parser:
    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SOF)
            if i < 0:
                self.buf.clear()
                break
            if i:
                del self.buf[:i]
            if len(self.buf) < 8:
                break
            plen = self.buf[5]
            if len(self.buf) < 8 + plen:
                break
            frame = bytes(self.buf[: 8 + plen])
            body = frame[2 : 6 + plen]
            crc = struct.unpack("<H", frame[6 + plen : 8 + plen])[0]
            if crc == crc16_ccitt_false(body):
                out.append((frame[3], frame[4], frame[6 : 6 + plen]))
                del self.buf[: 8 + plen]
            else:
                self.crc_errors += 1
                del self.buf[:2]
        return out


def flags81(f):
    s = []
    if f & 1: s.append("VALID")
    if f & 2: s.append("TIMEOUT")
    if f & 4: s.append("ALARM")
    return "|".join(s) or "none"


def flags85(f):
    s = []
    if f & 1: s.append("OUT_EN")
    if f & 2: s.append("CLOSED_LOOP")
    if f & 4: s.append("FLASH_OK")
    if f & 8: s.append("SAVE_OK")
    return "|".join(s) or "none"


def flags87(f):
    s = []
    if f & 1: s.append("ONLINE")
    if f & 2: s.append("CHARGING")
    if f & 4: s.append("CV")
    if f & 8: s.append("VIN")
    if f & 16: s.append("SEEN")
    return "|".join(s) or "none"


def flags88(f):
    s = []
    if f & 1: s.append("ENABLED")
    if f & 2: s.append("LIMIT")
    if f & 4: s.append("OUTPUT")
    if f & 8: s.append("TIMED_OUT")
    return "|".join(s) or "none"


def flags8a(f):
    s = []
    if f & 0x01: s.append("MAIN")
    if f & 0x02: s.append("AON")
    if f & 0x04: s.append("TEMP")
    if f & 0x08: s.append("CURR")
    if f & 0x10: s.append("VDDA_CAL")
    if f & 0x20: s.append("MG996_LIMIT")
    return "|".join(s) or "none"


def decode(ftype, seq, p):
    if ftype == 0x81 and len(p) == 12:
        cl, cr, al, ar, age, fl, rxseq = struct.unpack("<hhhhHBB", p)
        return f"81 MOTOR  cmd L={cl:5d} R={cr:5d}  pwm L={al:5d} R={ar:5d}  age={age:5d}ms  flags={flags81(fl)}  rxseq={rxseq}"
    if ftype == 0x82 and len(p) == 8:
        c, a, age, fl, rxseq = struct.unpack("<hhHBB", p)
        return f"82 LAWER  cmd={c:5d} pwm={a:5d} age={age:5d}ms flags={flags81(fl)} rxseq={rxseq}"
    if ftype == 0x83 and len(p) == 8:
        m, r, g, b, per, fl, rxseq = struct.unpack("<BBBBHBB", p)
        return f"83 WS2812 mode={m} rgb=({r},{g},{b}) period={per}ms flags={fl:#04x} rxseq={rxseq}"
    if ftype == 0x84 and len(p) == 28:
        lkp, lki, lkd, rkp, rki, rkd, fl, seqn, _ = struct.unpack("<ffffffBBH", p)
        return f"84 PID    L=({lkp:.2f},{lki:.2f},{lkd:.2f}) R=({rkp:.2f},{rki:.2f},{rkd:.2f}) flags={fl:#04x}"
    if ftype == 0x85 and len(p) == 24:
        lt, lm, rt, rm, lo, ro, ltot, rtot, fl = struct.unpack("<hhhhhhiiB3x", p)
        return (f"85 WHEEL  L tgt={lt/100:6.2f} meas={lm/100:6.2f} rpm out={lo:5d} tot={ltot:9d} | "
                f"R tgt={rt/100:6.2f} meas={rm/100:6.2f} rpm out={ro:5d} tot={rtot:9d} | {flags85(fl)}")
    if ftype == 0x86 and len(p) == 8:
        st, fl, reason, rxseq, press, elapsed = struct.unpack("<BBBBHH", p)
        return f"86 POWER  state={st} flags={fl:#04x} reason={reason} press={press}ms elapsed={elapsed}ms rxseq={rxseq}"
    if ftype == 0x87 and len(p) == 16:
        ma, mi, pa, proto, sha, built, bflags, blver, _ = struct.unpack("<BBBBIIBBH", p)
        when = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(built)) if built else "unknown"
        tags = ("dirty " if bflags & 0x01 else "") + ("unversioned" if bflags & 0x02 else "")
        return f"87 FWINFO v{ma}.{mi}.{pa} proto={proto} sha={sha:08x} built={when} {tags}".rstrip()
    if ftype == 0x89 and len(p) == 16:
        vin, vout, iout, cc, cv, fl, err, age, exc, _ = struct.unpack("<HHHHHBBHBB", p)
        age_s = "never" if age == 0xFFFF else f"{age}ms"
        return (f"89 CHARGE Vin={vin/100:5.2f}V Vout={vout/100:5.2f}V Iout={iout/100:4.2f}A "
                f"set CC={cc/100:4.2f}A CV={cv/100:5.2f}V  {flags87(fl)}  err={err} exc={exc} age={age_s}")
    if ftype == 0x88 and len(p) == 8:
        pulse, hold, age, fl, rxseq = struct.unpack("<HHHBB", p)
        hold_s = "forever" if hold == 0 else f"{hold}ms"
        return f"88 SERVO  pulse={pulse:4d}us hold={hold_s} age={age:5d}ms {flags88(fl)} rxseq={rxseq}"
    if ftype == 0x8A and len(p) == 12:
        vmain, vaon, temp, vdda, curr, fl, _ = struct.unpack("<HHhHHBB", p)
        temp_s = "n/a" if temp == -32768 else f"{temp/10:.1f}C"
        return (f"8A ANALOG main={vmain/100:5.2f}V aon={vaon/100:4.2f}V temp={temp_s} "
                f"vdda={vdda}mV mg996_raw={curr:4d}  {flags8a(fl)}")
    return f"{ftype:02X} seq={seq} payload={p.hex()}"


def run(port, cmd, args):
    ser = serial.Serial(port, 115200, timeout=0.02)
    parser = Parser()
    seq = 0
    duration = 3.0
    tx = None
    period = 0.05
    stop = None

    if cmd == "monitor":
        duration = float(args[0]) if args else 3.0
    elif cmd == "wheel":
        l, r = int(args[0]), int(args[1])
        duration = float(args[2]) if len(args) > 2 else 3.0
        tx = lambda s: build(0x01, s, struct.pack("<hhHH", l, r, 300, 0))
        stop = lambda s: build(0x01, s, struct.pack("<hhHH", 0, 0, 300, 0))
    elif cmd == "lawer":
        d = int(args[0])
        duration = float(args[1]) if len(args) > 1 else 3.0
        tx = lambda s: build(0x02, s, struct.pack("<hHHH", d, 300, 0, 0))
        stop = lambda s: build(0x02, s, struct.pack("<hHHH", 0, 300, 0, 0))
    elif cmd == "led":
        mode = int(args[0])
        r, g, b = (int(x) for x in args[1:4]) if len(args) >= 4 else (0, 0, 0)
        per = int(args[4]) if len(args) > 4 else 100
        duration = 1.0
        tx = lambda s: build(0x03, s, struct.pack("<BBBBHBB", mode, r, g, b, per, 0, 0))
        period = 0.2
    elif cmd == "info":
        duration = 1.0
        tx = lambda s: build(0x06, s, b"")
    elif cmd == "servo":
        pulse = int(args[0])
        hold = int(args[1]) if len(args) > 1 else 0
        duration = float(args[2]) if len(args) > 2 else 1.0
        f = build(0x07, 0, struct.pack("<HHHH", pulse, hold, 0, 0))
        tx = lambda s: f
        period = 0.5
    elif cmd == "pid":
        kp, ki = float(args[0]), float(args[1])
        kd = float(args[2]) if len(args) > 2 and not args[2].startswith("--") else 0.0
        save = 1 if "--save" in args else 0
        duration = 1.0
        f = build(0x04, 0, struct.pack("<ffffffBBH", kp, ki, kd, kp, ki, kd, save, 1, 0))
        tx = lambda s: f
        period = 0.5
    else:
        print(__doc__)
        return 2

    counts = {}
    t0 = time.time()
    last_tx = 0
    last_print = {}
    try:
        while time.time() - t0 < duration:
            if tx and time.time() - last_tx >= period:
                ser.write(tx(seq))
                seq = (seq + 1) & 0xFF
                last_tx = time.time()
            for ftype, fseq, p in parser.feed(ser.read(256)):
                counts[ftype] = counts.get(ftype, 0) + 1
                if time.time() - last_print.get(ftype, 0) >= 0.25:
                    print(f"{time.time()-t0:5.2f}s  {decode(ftype, fseq, p)}", flush=True)
                    last_print[ftype] = time.time()
    finally:
        if stop:
            ser.write(stop(seq))
        ser.close()
    print(f"-- frames: {{ {', '.join(f'{k:02X}:{v}' for k, v in sorted(counts.items()))} }}  crc_errors={parser.crc_errors}")
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    sys.exit(run(sys.argv[1], sys.argv[2], sys.argv[3:]))
