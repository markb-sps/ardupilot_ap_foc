import serial, struct, sys, time
PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
AMPS = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0   # keep < current_max (15A)

def crc16(d):
    c = 0
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c
def frame(p): return bytes([2, len(p)]) + p + struct.pack(">H", crc16(p)) + b"\x03"
def set_current(a): return frame(bytes([6]) + struct.pack(">i", int(a*1000)))  # COMM_SET_CURRENT
GET = frame(bytes([4]))

ser = serial.Serial(PORT, 115200, timeout=0.1)
def rd():
    # Resync: discard bytes until a short-packet start (0x02), bounded by the
    # serial timeout so a quiet link returns instead of spinning forever.
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

STATE = {0:"IDLE",1:"ALIGN",2:"OPENLOOP",3:"BLEND",4:"CLOSED",5:"FAULT",6:"DEBUG"}
try:
    while True:
        ser.write(set_current(AMPS)); ser.write(GET); time.sleep(0.05)   # NO firmware timeout — keep sending
        p = rd()
        if p and len(p) >= 86 and p[0] == 4:
            import math
            g = lambda o,s=1.0: struct.unpack_from(">i", p, o)[0] / s
            wrap = lambda a: (a + math.pi) % (2*math.pi) - math.pi
            theta = g(74,10000); obs = g(78,10000)
            # In OPENLOOP theta is the FORCED override angle; obs is the now
            # free-running observer (never seeded). lag = theta-obs is the
            # observer convergence error — must → ~0 (or a stable offset) by the
            # const phase for the trust-or-coast handover to fire. If it stays
            # ~150-180°, the observer isn't converging and the start will lock out.
            lag = wrap(theta - obs)
            print(f"{STATE.get(p[73],'?'):8} cmd={AMPS:+.1f}A  "
                  f"id={g(13,100):+.2f} iq={g(17,100):+.2f}  vq={g(69,1000):+.2f}  "
                  f"erpm={int(g(23)):+6d}  th={theta:+.2f} obs={obs:+.2f} "
                  f"lag={math.degrees(lag):+5.0f}deg  fault={p[53]}")
except KeyboardInterrupt:
    ser.write(set_current(0)); ser.close()   # 0A -> STOP -> coast