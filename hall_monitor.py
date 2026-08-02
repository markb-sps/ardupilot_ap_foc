#!/usr/bin/env python3
# hall_monitor.py — live raw hall-sensor state. Motor stays OFF; turn the rotor
# SLOWLY BY HAND and watch the state. A healthy 3-hall motor cycles through SIX
# distinct states once per electrical revolution, one bit changing at a time.
# Which six is motor-specific: many use {1,2,3,4,5,6}, others use {0,1,2,5,6,7}
# (0 and 7 are legal states for those). Only FEWER than six distinct states, or
# a channel that never moves independently, indicates a wiring/pin fault.
#
#   ./hall_monitor.py [port]
#
import serial, struct, sys, time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"

def crc16(d):
    c = 0
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c

def frame(p): return bytes([2, len(p)]) + p + struct.pack(">H", crc16(p)) + b"\x03"
GET = frame(bytes([4]))
ser = serial.Serial(PORT, 115200, timeout=0.1)

def rd():
    deadline = time.time() + 0.3
    while time.time() < deadline:
        b = ser.read(1)
        if not b: return None
        if b == b"\x02": break
    else:
        return None
    h = ser.read(1)
    if not h: return None
    n = h[0]; p = ser.read(n); tail = ser.read(3)
    return p if (len(p) == n and len(tail) == 3 and tail[2] == 0x03) else None

BITS = lambda s: f"C={(s>>2)&1} B={(s>>1)&1} A={s&1}"   # C=PB10 B=PB7 A=PB11
seen = set()
last = None
print("Turn the rotor slowly by hand. Ctrl-C to stop.\n")
try:
    while True:
        ser.write(GET); time.sleep(0.02)
        p = rd()
        if not p or p[0] != 4 or len(p) < 103:
            continue
        s = p[102]                     # appended live raw hall state (0..7)
        valid = p[103] if len(p) > 103 else -1  # hall table loaded & valid?
        if valid == 0 and last is None:
            print("NOTE: hall table NOT valid — HALL mode will refuse to drive "
                  "until you run hall_detect.py.")
        if s != last:
            seen.add(s)
            order = " ".join(str(x) for x in sorted(seen))
            print(f"hall={s} [{BITS(s)}]   seen so far: {{{order}}}")
            last = s
except KeyboardInterrupt:
    order = sorted(seen)
    print(f"\nDistinct states seen: {order}  ({len(order)} states)")
    if len(order) == 6:
        print("6 distinct states -> all three halls working. Run hall_detect.py.")
    elif len(order) < 6:
        print("Fewer than 6 states -> a hall channel isn't reading (wiring/pin).")
    else:
        print("More than 6 states -> noisy/bouncing reads; check pull-ups/shielding.")
    ser.close()
