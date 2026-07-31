#!/usr/bin/env python3
# bench_debug.py — open-loop voltage bring-up + raw ADC diagnostics
#
#   ./bench_debug.py [port] [duty|watch|beep|sweep|diag]
#
# "watch" is PASSIVE: polls GET_VALUES and sends NO set-point.
#
# "diag" is also passive — it asks the firmware for the arbiter / fault / TIM1
# state it can't fit into GET_VALUES, and commands nothing. Safe to run on a
# board sitting in FAULT, which is exactly when you need it.
#
# "beep" fires the firmware's tone on demand (terminal command) and watches the
# phase currents while it plays. The power-on chime fires before USB enumerates
# so it can never be caught over this link — hence triggering it here instead.
#
# "sweep" is the measurement: it plays the tone at several frequencies, records
# the peak phase current at each, and fits R and L from the result. One run
# gives everything.
#
#   Why this works: the tone drives phase A against B and C in PARALLEL (beep
#   sets m_beta = 0, so vb == vc), giving a series impedance of
#       |Z| = 1.5 * |R + jwL|
#   Sweeping w separates R from L, which a single DC-ish measurement cannot do:
#       (V / (1.5 * Ipk))^2 = R^2 + L^2 * w^2
#   is linear in w^2, so a least-squares line gives R^2 as the intercept and
#   L^2 as the slope. Rotor position doesn't matter — a fixed-axis AC vector
#   makes no net torque, so nothing turns and the geometry stays put.
import serial, struct, sys, time, math
PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
ARG  = sys.argv[2] if len(sys.argv) > 2 else "0.02"
SWEEP = ARG.lower() == "sweep"
DIAG  = ARG.lower() == "diag"
WATCH = ARG.lower() in ("watch", "beep", "diag") or SWEEP
BEEP  = ARG.lower() == "beep"
DUTY = 0.0 if WATCH else float(ARG)

# Sweep settings. Amplitude is capped by the firmware at debug_max_modulation
# (0.06); frequencies stay >=500 Hz because play_tone() floors at 200 Hz and
# low frequencies draw V/(1.5R) ~ several amps, and <=4 kHz because the 20 kHz
# PWM only samples ~5x per tone cycle above that.
SWEEP_FREQS = [500, 800, 1500, 2500, 4000]
SWEEP_AMP   = 0.06
SWEEP_MS    = 400     # tone length per shot
SWEEP_DWELL = 2.5     # seconds of polling per frequency
CFG_R, CFG_L = 0.055, 80e-6   # what the firmware is configured with, for comparison
def crc16(d):
    c = 0
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c
def frame(p): return bytes([2, len(p)]) + p + struct.pack(">H", crc16(p)) + b"\x03"
def set_duty(d): return frame(bytes([5]) + struct.pack(">i", int(d*100000)))
def terminal(s): return frame(bytes([20]) + s.encode())   # COMM_TERMINAL_CMD
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
    n = h[0]; p = ser.read(n); tail = ser.read(3)   # crc16(2) + end(0x03)
    return p if (len(p) == n and len(tail) == 3 and tail[2] == 0x03) else None
STATE = {0:"IDLE",1:"ALIGN",2:"OPENLOOP",3:"BLEND",4:"CLOSED",5:"FAULT",6:"DEBUG",
         7:"BEEP",8:"HALL",9:"HALL_DET"}
pk = [0.0, 0.0, 0.0]   # running peak |ia|,|ib|,|ic| — the chime is short
COMM_PRINT = 21        # firmware terminal text response

# MotorControl.h FaultCode. 30/31 are ours, outside VESC's range on purpose.
FAULTS = {0: "none", 2: "under-voltage", 4: "abs overcurrent",
          5: "over-temp FET", 30: "stall", 31: "hall sensor"}


def diag():
    """Run the firmware's `diag` terminal command and print what comes back.

    print_diag() answers the questions GET_VALUES structurally can't: which
    throttle source the arbiter picked, whether the bridge is actually armed
    (MOE) versus merely believed to be (_outputs_on), and what the three CCRs
    are — three equal CCRs mean common mode only, i.e. no torque whatever the
    state field says. Sends no set-point, so it is safe on a faulted board.
    """
    ser.reset_input_buffer()
    ser.write(terminal("diag"))
    deadline, lines = time.time() + 1.5, []
    while time.time() < deadline:
        p = rd()
        if p and p[0] == COMM_PRINT:
            lines.append(p[1:].decode("ascii", "replace").rstrip())
            deadline = time.time() + 0.4   # more lines follow; extend while they do
    return lines


def measure_freq(f_hz):
    """Play the tone at f_hz and return (peak|ia|, peak|ib|, peak|ic|, vbus, n).

    Only samples taken while the firmware reports State::BEEP (7) count — idle
    samples in the gaps between tone shots would otherwise drag the peak down
    with pure offset noise. The tone is re-fired before the previous one ends so
    it plays continuously across the dwell.
    """
    peaks = [0.0, 0.0, 0.0]
    vbus, n = 0.0, 0
    next_shot, t_end = 0.0, time.time() + SWEEP_DWELL
    while time.time() < t_end:
        now = time.time()
        if now >= next_shot:
            ser.write(terminal(f"beep {f_hz} {SWEEP_AMP} {SWEEP_MS}"))
            next_shot = now + (SWEEP_MS / 1000.0) * 0.85   # overlap, no silent gaps
        ser.write(GET); time.sleep(0.02)
        p = rd()
        if not (p and len(p) >= 104 and p[0] == 4 and p[73] == 7):
            continue
        g = lambda o, s=1.0: struct.unpack_from(">i", p, o)[0] / s
        for i, off in enumerate((86, 90, 94)):
            v = abs(g(off, 100))
            if v > peaks[i]:
                peaks[i] = v
        vbus = struct.unpack_from(">h", p, 27)[0] / 10.0   # v_in, f16 scaled x10
        n += 1
    return peaks[0], peaks[1], peaks[2], vbus, n


def fit_rl(points):
    """Least-squares fit of z^2 = R^2 + L^2 * w^2 over (w, z) samples."""
    xs = [w * w for w, _ in points]
    ys = [z * z for _, z in points]
    n  = len(xs)
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    den = n * sxx - sx * sx
    if den == 0:
        return None, None
    slope     = (n * sxy - sx * sy) / den
    intercept = (sy - slope * sx) / n
    R = math.sqrt(intercept) if intercept > 0 else 0.0
    L = math.sqrt(slope) if slope > 0 else 0.0
    return R, L


if DIAG:
    lines = diag()
    if not lines:
        print("no reply to `diag` — check the port, and that this build has the "
              "FOC ESC telemetry (COMM_TERMINAL_CMD) enabled.")
        sys.exit(1)
    for s in lines:
        print("  " + s)
        # Spell out the fault number; it is the whole reason for running this.
        for tok in s.split():
            if tok.startswith("fault="):
                code = int(tok[6:])
                print(f"    -> fault {code}: {FAULTS.get(code, 'UNKNOWN')}")
    print("\nnote: `fault=` is the LIVE code (MotorControl::get_fault()), so a trip\n"
          "that has since cleared reads 0 here even though the run showed st=FAULT.")
    sys.exit(0)


if SWEEP:
    print("Tone sweep: measuring |Z| at each frequency, then fitting R and L.\n"
          f"amp {SWEEP_AMP}, {SWEEP_DWELL:.1f} s per point. Motor will buzz, not turn.\n")
    print(f"{'f [Hz]':>7} {'ipk_a':>7} {'ipk_b':>7} {'ipk_c':>7} {'b/c':>6} "
          f"{'V [V]':>7} {'|Z| [ohm]':>10} {'n':>4}")
    pts, rows = [], []
    for f in SWEEP_FREQS:
        ia_pk, ib_pk, ic_pk, vbus, n = measure_freq(f)
        if n == 0 or ia_pk <= 0.0:
            print(f"{f:>7} {'--':>7} {'--':>7} {'--':>7} {'--':>6} "
                  f"{'--':>7} {'no BEEP samples':>10} {n:>4}")
            continue
        v_cmd = SWEEP_AMP * vbus / 2.0        # commanded phase-A amplitude [V]
        z1    = v_cmd / (1.5 * ia_pk)         # |R + jwL|, the 1.5 removes B||C
        ratio = (ib_pk / ic_pk) if ic_pk > 0 else float('inf')
        print(f"{f:>7} {ia_pk:>7.2f} {ib_pk:>7.2f} {ic_pk:>7.2f} {ratio:>6.2f} "
              f"{v_cmd:>7.3f} {z1:>10.4f} {n:>4}")
        pts.append((2.0 * math.pi * f, z1))
        rows.append((ib_pk, ic_pk))
    ser.write(terminal("beep 1000 0.0 1"))    # amp 0 -> firmware default, harmless
    if len(pts) >= 2:
        R, L = fit_rl(pts)
        print(f"\nfitted:     R = {R*1000:.1f} m-ohm    L = {L*1e6:.1f} uH")
        print(f"configured: R = {CFG_R*1000:.1f} m-ohm    L = {CFG_L*1e6:.1f} uH")
        if L > 0:
            print(f"            L is {CFG_L/L:.2f}x off -> current-loop Kp "
                  f"(= bw * L) is off by the same factor")
        if rows:
            avg = sum(b / c for b, c in rows if c > 0) / max(1, len([1 for _, c in rows if c > 0]))
            print(f"\nphase B/C peak ratio averaged {avg:.2f} (should be 1.00 — "
                  f"beep sets vb == vc, so B and C sit in parallel)")
    else:
        print("\nnot enough points to fit — did the tone play? check for a latched fault")
    ser.close()
    sys.exit(0)

if BEEP:
    # Re-fire periodically so the tone is always playing while polling; the
    # firmware caps duration and refuses while a fault is latched.
    print("BEEP mode: firing a 2 kHz tone every 600 ms and watching the phase\n"
          "currents. Peaks are cumulative; Ctrl-C to stop.\n")
    next_beep = 0.0
elif WATCH:
    print("PASSIVE watch: sending no set-point. Peaks are cumulative; Ctrl-C to stop.\n")
try:
    while True:
        if BEEP and time.time() >= next_beep:
            ser.write(terminal("beep 2000 0.06 500"))
            next_beep = time.time() + 0.6
        if not WATCH:
            ser.write(set_duty(DUTY))
        ser.write(GET); time.sleep(0.05)
        p = rd()
        if p and len(p) >= 104 and p[0] == 4:
            import math
            g = lambda o,s=1.0: struct.unpack_from(">i", p, o)[0] / s
            erpm  = g(23)
            theta = g(74,10000); obs = g(78,10000)
            # All THREE phase currents are measured (third shunt on ADC1 rank 2)
            # — do NOT reconstruct ic as -(ia+ib), which was hiding the fault it
            # exists to expose. `res` is the firmware's own ia+ib+ic.
            #
            # Kirchhoff holds on a 3-wire motor no matter which legs conduct, so
            # res is the discriminator between a dead leg and a dead sensor:
            #   res ~ 0  -> all three senses good; a near-zero phase current is
            #              REAL (open winding / dead FET / dead gate driver)
            #   res != 0 -> a current SENSE is broken; the motor may be fine
            ia = g(86,100); ib = g(90,100); ic = g(94,100)
            res = g(98,100)
            # Angle the observer lags the commanded angle by (load angle), wrapped.
            lag = (theta - obs + math.pi) % (2*math.pi) - math.pi
            for i, v in enumerate((ia, ib, ic)):
                if abs(v) > pk[i]:
                    pk[i] = abs(v)
            st = STATE.get(p[73], str(p[73]))
            print(f"st={st:8} id={g(13,100):+.2f} iq={g(17,100):+.2f}  "
                  f"ia={ia:+.2f} ib={ib:+.2f} ic={ic:+.2f} res={res:+.2f}  "
                  f"pk={pk[0]:.2f}/{pk[1]:.2f}/{pk[2]:.2f}  "
                  f"erpm={erpm:+7.0f}  theta={theta:+.2f} obs={obs:+.2f} "
                  f"lag={math.degrees(lag):+5.0f}deg  vd={g(65,1000):+.2f} "
                  f"hall={p[102]} tab={p[103]}")
        elif p and p[0] == 4:
            # Short packet = firmware predating the appended bring-up fields.
            print(f"packet too short ({len(p)} B, need 104) — reflash this board")
except KeyboardInterrupt:
    if not WATCH:
        ser.write(set_duty(0))   # watch mode never commanded anything — stay passive
    print(f"\npeak |ia|/|ib|/|ic| = {pk[0]:.2f}/{pk[1]:.2f}/{pk[2]:.2f} A")
    ser.close()
