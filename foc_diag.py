#!/usr/bin/env python3
# foc_diag.py — dump the FOC throttle-arbiter state via the VESC terminal.
#
# The arbiter picks, in order: chime -> override -> CAN -> USB -> PWM -> coast.
# A motor sitting in IDLE with fault=0 means an earlier branch is winning (or
# the whole block is skipped), and none of that is visible in COMM_GET_VALUES.
#
# Optionally sends a COMM_SET_CURRENT first, so the USB source is fresh at the
# moment the diag is taken -- otherwise usb_fresh is trivially false.
#
#   ./foc_diag.py [port] [value] [mode]
#
# mode "current" (default) holds a COMM_SET_CURRENT setpoint; mode "duty" holds
# an open-loop COMM_SET_DUTY instead. Use "duty" when the hall table is invalid
# (hall_tab=0) — current mode is refused outright in that state, so the bridge
# would be idle at the moment the diag is taken and the read-back meaningless.
#
import serial, struct, sys, time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
VAL  = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
MODE = sys.argv[3].lower() if len(sys.argv) > 3 else "current"

COMM_SET_DUTY     = 5
COMM_SET_CURRENT  = 6
COMM_TERMINAL_CMD = 20
COMM_PRINT        = 21


def crc16(d):
    c = 0
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def frame(p):
    return bytes([2, len(p)]) + p + struct.pack(">H", crc16(p)) + b"\x03"


def set_current(a):
    return frame(bytes([COMM_SET_CURRENT]) + struct.pack(">i", int(a * 1000)))


def set_duty(d):
    return frame(bytes([COMM_SET_DUTY]) + struct.pack(">i", int(d * 100000)))


hold    = (lambda: set_duty(VAL))  if MODE == "duty" else (lambda: set_current(VAL))
release = (lambda: set_duty(0.0))  if MODE == "duty" else (lambda: set_current(0.0))


def terminal(s):
    return frame(bytes([COMM_TERMINAL_CMD]) + s.encode())


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
            n = h[0]
            p = ser.read(n)
            tail = ser.read(3)   # crc16(2) + end(0x03)
            if len(p) == n and len(tail) == 3 and tail[2] == 0x03:
                return p
    return None


# Keep the USB source fresh for ~300 ms so usb_current() has something to report,
# then ask for the diag while that setpoint is still inside the 200 ms window.
unit = "duty" if MODE == "duty" else "A"
print(f"holding {VAL:+.3f} {unit} on the USB source, then reading diag...\n")
end = time.time() + 0.3
while time.time() < end:
    ser.write(hold())
    time.sleep(0.05)

ser.write(hold())
ser.write(terminal("diag"))

deadline = time.time() + 2.0
lines = 0
while time.time() < deadline and lines < 12:
    p = rd(deadline)
    if p and len(p) >= 2 and p[0] == COMM_PRINT:
        print("  " + p[1:].decode(errors="replace").rstrip("\x00\r\n"))
        lines += 1

if lines == 0:
    print("  no reply -- is this the FOC_ESC firmware, and does it have 'diag'?")

ser.write(release())
ser.close()
