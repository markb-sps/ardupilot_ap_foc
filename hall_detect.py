#!/usr/bin/env python3
# hall_detect.py — trigger the firmware's hall-sensor table detection spin and
# print the resulting per-state electrical angles.
#
# The motor MUST be free to spin and unloaded; the spin is open-loop at low
# modulation (~2 s). Uses the same VESC command VESC Tool's hall detect sends
# (COMM_DETECT_HALL_FOC = 28), so this and VESC Tool are interchangeable.
#
#   ./hall_detect.py [port] [detect_current_A]
#
import serial, struct, sys, time

PORT    = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
CURRENT = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
COMM_DETECT_HALL_FOC = 28   # real VESC COMM_PACKET_ID (datatypes.h)

def crc16(d):
    c = 0
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c

def frame(p): return bytes([2, len(p)]) + p + struct.pack(">H", crc16(p)) + b"\x03"

# [25, float32 current*1e3]. The current arg is sent for VESC-Tool compatibility
# but this firmware ignores it (fixed low-modulation spin).
def detect(current): return frame(bytes([COMM_DETECT_HALL_FOC]) + struct.pack(">i", int(current * 1000)))

ser = serial.Serial(PORT, 115200, timeout=0.1)

def rd(deadline):
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b == b"\x02":
            h = ser.read(1)
            if not h:
                return None
            n = h[0]; p = ser.read(n); tail = ser.read(3)   # crc16(2) + end(0x03)
            if len(p) == n and len(tail) == 3 and tail[2] == 0x03:
                return p
    return None

print("Starting hall detect — keep the motor FREE TO SPIN and unloaded...")
ser.write(detect(CURRENT))

p = None
end = time.time() + 6.0
while time.time() < end:
    q = rd(end)
    if q and len(q) >= 10 and q[0] == COMM_DETECT_HALL_FOC:
        p = q
        break

if p is None:
    print("No detection reply. Is the FOC_ESC firmware running on this port?")
    ser.close(); sys.exit(1)

tab = p[1:9]; res = p[9]
deg = ["inv" if v == 255 else round(v / 200 * 360) for v in tab]
print("hall table (electrical deg, by hall state 0..7):", deg)
print("  'inv' = a state this motor never enters (the 2 of 8 it skips)")
print("result:", "OK — six states mapped" if res == 0 else "FAILED (fewer than 6 states — did it spin freely?)")
vals = ", ".join("NAN" if d == "inv" else str(d) for d in deg)
print("\nTo persist, set in FOC_ESC::init() (Tools/AP_Periph/foc_esc.cpp):")
print(f"  motor_cfg.hall_table_deg = {{ {vals} }};")
ser.close()
