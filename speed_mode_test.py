#!/usr/bin/env python3
# bench_rpm.py — speed test: PORT [erpm]
import serial, struct, sys, time
PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
ERPM = float(sys.argv[2]) if len(sys.argv) > 2 else 2000.0
def crc16(d):
    c = 0
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c
def frame(p): return bytes([2, len(p)]) + p + struct.pack(">H", crc16(p)) + b"\x03"
def set_rpm(e): return frame(bytes([8]) + struct.pack(">i", int(e)))   # COMM_SET_RPM
GET = frame(bytes([4]))
ser = serial.Serial(PORT, 115200, timeout=0.1)
def rd():
    # Resync: discard bytes until a short-packet start (0x02), bounded by timeout.
    deadline = time.time() + 0.3
    while time.time() < deadline:
        b = ser.read(1)
        if not b: return None
        if b == b"\x02": break
    else:
        return None
    h = ser.read(1)
    if not h: return None
    n = h[0]; p = ser.read(n); t = ser.read(3)
    return p if (len(p)==n and len(t)==3 and t[2]==0x03) else None
STATE={0:"IDLE",1:"ALIGN",2:"OPENLOOP",3:"BLEND",4:"CLOSED",5:"FAULT",6:"DEBUG"}
try:
    while True:
        ser.write(set_rpm(ERPM)); ser.write(GET); time.sleep(0.05)
        p = rd()
        if p and len(p) >= 86 and p[0] == 4:
            import math
            g = lambda o,s=1.0: struct.unpack_from(">i", p, o)[0]/s
            wrap = lambda a: (a + math.pi) % (2*math.pi) - math.pi
            theta = g(74,10000); obs = g(78,10000); free = g(82,10000)
            ferr = wrap(theta - free)
            print(f"{STATE.get(p[73],'?'):8} tgt={ERPM:+.0f}  erpm={int(g(23)):+6d}  "
                  f"iq={g(17,100):+.2f} id={g(13,100):+.2f}  vq={g(69,1000):+.2f}  "
                  f"th={theta:+.2f} obs={obs:+.2f} free={free:+.2f} ferr={math.degrees(ferr):+5.0f}deg  fault={p[53]}")
except KeyboardInterrupt:
    ser.write(set_rpm(0)); ser.close()
