#include "foc_esc.h"

#ifdef HAL_PERIPH_ENABLE_FOC_ESC

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL_ChibiOS/stm32_pwm_input.h>

extern const AP_HAL::HAL& hal;

// ── Current-sense scale (v2 PCB hardware) ───────────────────────────────────
// Exact scale = 1 / (Rshunt · input_attenuation · amp_gain).
namespace {
constexpr float phase_current_shunt_ohms = 0.001f;  // R407 = 1 mΩ
// External INA181A1 (gain 20 V/V) sits directly across the shunt — no external
// input divider. Its REF = 1.8 V zero-current bias is removed by the firmware's
// startup auto-calibration, so it doesn't enter the scale.
constexpr float phase_current_amp_gain = 20.0f;
constexpr float phase_current_shunt_input_attenuation = 1.0f;

// ── J305 RC PWM throttle calibration (single-channel servo pulse) ───────────
// Pistol-grip / trigger transmitter: neutral is mid-travel, push forward for
// throttle, pull back for brake. Mirrors VESC's PPM_CTRL_TYPE_CURRENT_NOREV_BRAKE
// (app_ppm.c:314) — brake only, never reverse.
constexpr uint16_t THR_PWM_MIN_US       = 1000;  // full brake pulse
constexpr uint16_t THR_PWM_CTR_US       = 1500;  // neutral (VESC's pulse_center)
constexpr uint16_t THR_PWM_MAX_US       = 2000;  // full throttle pulse
// Neutral deadband either side of centre (VESC's ppm_config.hyst). A trigger
// spring does not return to the same microsecond twice; without this the motor
// creeps or drags at rest. Adjust the transmitter's subtrim to centre the
// trigger inside this band — that is the calibration, not a firmware change.
constexpr uint16_t THR_PWM_DEADBAND_US  = 40;
constexpr uint16_t THR_PWM_RANGE_TOL_US = 100;   // accept 900..2100; beyond → signal invalid
// Arming gate: the trigger must be seen at NEUTRAL before it may command
// anything, so a power-up or reconnect with the trigger held neither launches
// the motor nor slams the brake on. Deliberately not the old "≤1000 µs" gate:
// on a pistol grip 1000 µs is full brake, a position the trigger never rests
// in, so that test would leave the PWM source permanently disarmed.
// ── Reverse-roll threshold [eRPM] ───────────────────────────────────────────
// Below this the motor counts as "not rolling backwards" and a forward trigger
// gets full motoring current. VESC tests `rpm_now > 0.0` (app_ppm.c:318), which
// at a standstill takes the brake branch and launches at l_current_min — benign
// there only because stock VESC configs set l_current_min = -l_current_max. Ours
// is deliberately asymmetric (I_REGEN 5 A vs I_MAX 15 A), so copying that test
// verbatim would cap every launch at a third of available torque. A small
// deadband instead of `>= 0` keeps it robust to PLL noise around zero.
constexpr float    THR_REV_ERPM         = 100.0f;

// A source is "fresh" for this long after its last valid command — covers a
// couple of dropped 50 Hz RC frames / DroneCAN commands before it ages to coast.
constexpr uint32_t THR_SOURCE_TIMEOUT_MS = 200;

// VESC ppm_control_type values (bldc datatypes.h). Only these three are
// implemented; the NUMBERING is VESC's so a value stored here is the same
// integer as the index of VESC Tool's Control Type dropdown, and a config
// exchanged with the tool needs no translation in either direction.
constexpr uint8_t PPM_CTRL_CURRENT              = 1;  // "Current"
constexpr uint8_t PPM_CTRL_CURRENT_NOREV_BRAKE  = 3;  // "Current No Reverse With Brake"
constexpr uint8_t PPM_CTRL_PID                  = 6;  // "PID Speed Control"
constexpr uint8_t PPM_CTRL_PID_NOREV            = 7;  // "PID Speed Control No Reverse"
constexpr uint8_t PPM_CTRL_CURRENT_BRAKE_REV_HYST = 8;  // "Current Hyst Reverse With Brake"

// True for the control types whose idle stick is at MINIMUM pulse rather than
// centre, because they map the trigger to a one-sided 0..1 command (VESC
// app_ppm.c:144-151, `servo_val = (servo_val + 1) / 2`). Both the arming gate
// and the throttle mapping have to agree on where "no command" lives, or a
// NOREV type would arm at centre — which is already half throttle.
static inline bool ppm_ctrl_is_norev(uint8_t ctrl)
{
    return ctrl == PPM_CTRL_PID_NOREV;
}

// True for the control types that command a SPEED rather than a current.
static inline bool ppm_ctrl_is_pid(uint8_t ctrl)
{
    return ctrl == PPM_CTRL_PID || ctrl == PPM_CTRL_PID_NOREV;
}

// ── Power-on chime (played through the motor windings) ──────────────────────
struct ChimeNote { uint16_t freq_hz; uint16_t ms; };
constexpr ChimeNote CHIME[] = { {1047, 120}, {1319, 120}, {1568, 200} };  // C6–E6–G6
constexpr uint8_t   CHIME_LEN       = sizeof(CHIME) / sizeof(CHIME[0]);
constexpr float     CHIME_AMPLITUDE = 0.08f;  // modulation fraction (capped in play_tone)
constexpr uint16_t  CHIME_GAP_MS    = 40;     // silence between notes
// How long to wait for the controller to become ready before giving up and
// skipping the chime. Must not be indefinite — see update_startup_chime().
constexpr uint32_t  CHIME_READY_TIMEOUT_MS = 3000;
}

const AP_Param::GroupInfo FOC_ESC::var_info[] = {
    // @Param: SENSOR
    // @DisplayName: FOC rotor-angle sensor mode
    // @Description: Rotor angle source for the FOC ESC.
    // @Values: 0:Sensorless,1:Hall
    // @User: Standard
    AP_GROUPINFO("SENSOR", 1, FOC_ESC, _p_sensor_mode, 1),

    // @Param: HALL0
    // @DisplayName: Hall table angle for state 0
    // @Description: Electrical angle for hall state 0, or -1 if this state is unused. Set by hall detection.
    // @Units: deg
    // @Range: -1 359
    // @User: Advanced
    AP_GROUPINFO("HALL0", 2, FOC_ESC, _p_hall[0], -1),
    // @Param: HALL1
    // @Description: Hall table angle for state 1 (deg, -1 = unused).
    AP_GROUPINFO("HALL1", 3, FOC_ESC, _p_hall[1], -1),
    // @Param: HALL2
    // @Description: Hall table angle for state 2 (deg, -1 = unused).
    AP_GROUPINFO("HALL2", 4, FOC_ESC, _p_hall[2], -1),
    // @Param: HALL3
    // @Description: Hall table angle for state 3 (deg, -1 = unused).
    AP_GROUPINFO("HALL3", 5, FOC_ESC, _p_hall[3], -1),
    // @Param: HALL4
    // @Description: Hall table angle for state 4 (deg, -1 = unused).
    AP_GROUPINFO("HALL4", 6, FOC_ESC, _p_hall[4], -1),
    // @Param: HALL5
    // @Description: Hall table angle for state 5 (deg, -1 = unused).
    AP_GROUPINFO("HALL5", 7, FOC_ESC, _p_hall[5], -1),
    // @Param: HALL6
    // @Description: Hall table angle for state 6 (deg, -1 = unused).
    AP_GROUPINFO("HALL6", 8, FOC_ESC, _p_hall[6], -1),
    // @Param: HALL7
    // @Description: Hall table angle for state 7 (deg, -1 = unused).
    AP_GROUPINFO("HALL7", 9, FOC_ESC, _p_hall[7], -1),

    // ── Motor identity (VESC Tool "Write Motor Configuration" writes these) ──
    // @Param: M_RS
    // @DisplayName: Motor phase resistance
    // @Units: Ohm
    // @User: Standard
    AP_GROUPINFO("M_RS", 10, FOC_ESC, _p_motor_rs, 0.055f),
    // @Param: M_LS
    // @DisplayName: Motor phase inductance
    // @Units: H
    // @User: Standard
    AP_GROUPINFO("M_LS", 11, FOC_ESC, _p_motor_ls, 80e-6f),
    // @Param: M_FLUX
    // @DisplayName: Motor PM flux linkage
    // @Units: Wb
    // @User: Standard
    AP_GROUPINFO("M_FLUX", 12, FOC_ESC, _p_motor_flux, 4.6e-3f),
    // @Param: M_POLES
    // @DisplayName: Motor pole pairs
    // @Range: 1 30
    // @User: Standard
    AP_GROUPINFO("M_POLES", 13, FOC_ESC, _p_motor_poles, 7),

    // ── Current loop / limits ────────────────────────────────────────────────
    // @Param: I_MAX
    // @DisplayName: Max motor current (iq ceiling)
    // @Units: A
    // @User: Standard
    AP_GROUPINFO("I_MAX", 14, FOC_ESC, _p_i_max, 15.0f),
    // @Param: I_OC
    // @DisplayName: Overcurrent trip (debounced, per phase)
    // @Units: A
    // @User: Advanced
    AP_GROUPINFO("I_OC", 15, FOC_ESC, _p_i_oc, 30.0f),
    // @Param: I_OCHARD
    // @DisplayName: Instant overcurrent trip (per phase)
    // @Units: A
    // @User: Advanced
    AP_GROUPINFO("I_OCHARD", 16, FOC_ESC, _p_i_oc_hard, 45.0f),
    // @Param: I_REGEN
    // @DisplayName: Max braking/regen current
    // @Units: A
    // @User: Advanced
    AP_GROUPINFO("I_REGEN", 17, FOC_ESC, _p_i_regen, 5.0f),
    // @Param: I_SLEW
    // @DisplayName: Torque-command slew limit
    // @Units: A/s
    // @User: Advanced
    AP_GROUPINFO("I_SLEW", 18, FOC_ESC, _p_i_slew, 150.0f),
    // @Param: I_SCALE
    // @DisplayName: Current-sense scale (ADC volts to amps)
    // @User: Advanced
    AP_GROUPINFO("I_SCALE", 19, FOC_ESC, _p_i_scale, 50.0f),

    // ── Bus / thermal / stall protections ────────────────────────────────────
    // @Param: V_MAX
    // @DisplayName: Bus voltage at which regen is fully cut
    // @Description: Braking current folds linearly to zero as the bus rises from (V_MAX - V_FOLD) to V_MAX. VESC calls the pair l_battery_regen_cut_start/_end, which is where VESC Tool shows them. Set a few volts above the bus you actually run at: too high and regen pumps the bus with nothing trimming it, too low and normal braking is derated away.
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("V_MAX", 20, FOC_ESC, _p_v_max, 40.0f),
    // @Param: V_FOLD
    // @DisplayName: Bus over-voltage foldback band
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("V_FOLD", 21, FOC_ESC, _p_v_fold, 3.0f),
    // @Param: D_MAX
    // @DisplayName: Per-phase duty ceiling
    // @Description: HARDWARE limit, not a tuning preference. The MP1918 high-side bootstrap only recharges while the low side conducts, and the phase-shunt ADC samples in that same window (valid to ~0.93 duty at 20 kHz per stm32_foc_motor_control.cpp). Raising it raises top speed roughly linearly (it sets the modulation ceiling, 2*(D_MAX-0.5)), and at some point drops a high-side FET out of enhancement while it carries current. Clamped to 0.55..0.90 on load, i.e. a modulation ceiling of 0.10..0.80. NOTE VESC Tool's "Maximum Duty Cycle" box is the MODULATION figure, not this one: the link converts both ways, so 80% there stores D_MAX 0.90 here.
    // @Range: 0.55 0.90
    // @User: Advanced
    AP_GROUPINFO("D_MAX", 36, FOC_ESC, _p_duty_max, 0.80f),
    // @Param: D_START
    // @DisplayName: Duty at which current foldback starts
    // @Description: Fraction of the modulation ceiling at which the current limit begins tapering, reaching near-zero at the ceiling (VESC l_duty_start, "Duty Cycle Current Limit Start"). This is what makes the motor settle at base speed instead of accelerating into voltage saturation, where the loop loses authority and back-EMF overshoot becomes tens of amps of braking current. Above 0.99 disables it, which is VESC's default; ours is on because this board's duty ceiling puts base speed inside the usable throttle range.
    // @Range: 0.30 1.0
    // @User: Advanced
    AP_GROUPINFO("D_START", 45, FOC_ESC, _p_duty_start, 0.85f),
    // @Param: MAX_ERPM
    // @DisplayName: Forward speed ceiling
    // @Description: Motoring current tapers from full at ERPM_START*MAX_ERPM to zero at MAX_ERPM electrical RPM (VESC l_max_erpm, "Max ERPM"). Understand what this is before relying on it: a torque CUTBACK, not a speed controller. Above the ceiling the drive stops pushing but never brakes, so momentum or a load driving the motor carries the speed straight past it. It bounds what the drive will ACCELERATE to, nothing more. For a speed that is actually held, use SPEED mode. Default 100000 is VESC's, i.e. effectively off.
    // @Units: rpm
    // @User: Advanced
    AP_GROUPINFO("MAX_ERPM", 46, FOC_ESC, _p_max_erpm, 100000.0f),
    // @Param: MIN_ERPM
    // @DisplayName: Reverse speed ceiling
    // @Description: The mirror of MAX_ERPM for reverse rotation, and NEGATIVE (VESC l_min_erpm, "Min ERPM"). Both ceilings act on the MOTORING current only, never on braking, so this limits how fast the drive will drive itself backwards; it does not limit how fast it may be pushed backwards. Default -100000 is VESC's, i.e. effectively off.
    // @Units: rpm
    // @User: Advanced
    AP_GROUPINFO("MIN_ERPM", 47, FOC_ESC, _p_min_erpm, -100000.0f),
    // @Param: ERPM_START
    // @DisplayName: ERPM at which current foldback starts
    // @Description: Fraction of MAX_ERPM/MIN_ERPM at which the current ceiling begins tapering (VESC l_erpm_start, "ERPM Limit Start"). Lower makes the limit softer and engage earlier. Clamped to 0.05..0.99 on load: unlike VESC, 1.0 is NOT accepted, because a zero-width taper here would disable the limit rather than harden it.
    // @Range: 0.05 0.99
    // @User: Advanced
    AP_GROUPINFO("ERPM_START", 48, FOC_ESC, _p_erpm_start, 0.8f),
    // @Param: PPM_CTRL
    // @DisplayName: PWM throttle control type
    // @Description: How the RC PWM throttle input is interpreted (VESC app_ppm_conf.ctrl_type, "Control Type" on VESC Tool's App Settings PPM page). Values are VESC's enum, so this matches that dropdown's index. Five are implemented. Torque types: 1 = Current (seamless bidirectional — back-stick brakes and then accelerates in reverse through zero with no neutral visit, so a held back-stick WILL launch backwards once the motor stops), 3 = Current No Reverse With Brake (centre is off, back-stick brakes to a stop and no further), 8 = Current Hyst Reverse With Brake (as 3, but braking to a stop, releasing to centre, then braking again enters reverse — gated by DIR_ERPM). Speed types, which close the outer speed loop on the trigger and are scaled by PID_ERPM and tuned by the SPD_ parameters: 6 = PID Speed Control (centre is stop, full travel each way is plus or minus PID_ERPM), 7 = PID Speed Control No Reverse (MINIMUM stick is stop, full travel is PID_ERPM forward, and the arming gate moves to the minimum stick to match). Writing any other value from VESC Tool is refused rather than silently substituted.
    // @Values: 1:Current,3:Current No Reverse With Brake,6:PID Speed Control,7:PID Speed Control No Reverse,8:Current Hyst Reverse With Brake
    // @User: Advanced
    AP_GROUPINFO("PPM_CTRL", 49, FOC_ESC, _p_ppm_ctrl, float(PPM_CTRL_CURRENT_NOREV_BRAKE)),
    // @Param: DIR_ERPM
    // @DisplayName: Max ERPM for direction switch
    // @Description: Above this electrical RPM the brake-to-reverse gesture is refused and back-stick stays a pure brake (VESC app_ppm_conf.max_erpm_for_dir, "Max ERPM for direction switch"; the param name is shortened to fit the ArduPilot 16-character limit). A 20 percent band around it provides hysteresis, so the gesture re-enables at 0.8x and disables again at 1.2x. Only used when PPM_CTRL is 8.
    // @Units: rpm
    // @User: Advanced
    AP_GROUPINFO("DIR_ERPM", 50, FOC_ESC, _p_dir_erpm, 4000.0f),

    // ── Speed control (VESC app_ppm_conf.pid_max_erpm + the s_pid_* group) ────
    // Indices 51-58 previously held HALLW0-7, which were AP_Int16 and have been
    // removed. Reusing the indices is safe because AP_Param matches a stored
    // record on TYPE as well as key and index (AP_Param::scan), so the orphaned
    // int16 records cannot be read back as these floats — a board carrying them
    // simply falls through to the defaults below. 59-62 are still virgin, and 63
    // is the group terminator, so that is all the room left in this group.
    // @Param: PID_ERPM
    // @DisplayName: Full-throttle speed setpoint
    // @Description: Speed commanded at full trigger when PPM_CTRL selects one of the PID speed control types (VESC app_ppm_conf.pid_max_erpm, "Max ERPM" on VESC Tool's App Settings PPM page). The trigger maps linearly onto plus/minus this for control type 6 and zero to this for type 7. The speed loop additionally truncates its setpoint to MIN_ERPM..MAX_ERPM, so setting this beyond those ceilings just saturates there rather than commanding a speed the drive is configured to refuse.
    // @Units: rpm
    // @Range: 0 100000
    // @User: Standard
    AP_GROUPINFO("PID_ERPM", 51, FOC_ESC, _p_pid_erpm, 15000.0f),
    // @Param: SPD_KP
    // @DisplayName: Speed PID P gain
    // @Description: Proportional gain of the outer speed loop (VESC s_pid_kp). The loop is a direct port of VESC's, including its internal 1/20 scale factor and its normalised plus/minus 1 output, so this number is exactly the one VESC Tool shows and gains transfer between the two without conversion. Output of 1 means the full I_MAX ceiling. Tune this first with SPD_KI at zero on a speed step, then add I.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("SPD_KP", 52, FOC_ESC, _p_spd_kp, 0.004f),
    // @Param: SPD_KI
    // @DisplayName: Speed PID I gain
    // @Description: Integral gain of the outer speed loop (VESC s_pid_ki). Exactly zero disables the integral term and holds it cleared, matching VESC. The integral is clamped to the full plus/minus 1 output range rather than back-calculated; VESC relies on SPD_RAMP keeping the error small enough that it never runs away, so a very slow ramp with a large step command is the case to watch.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("SPD_KI", 53, FOC_ESC, _p_spd_ki, 0.004f),
    // @Param: SPD_KD
    // @DisplayName: Speed PID D gain
    // @Description: Derivative gain of the outer speed loop (VESC s_pid_kd). Acts on the speed ERROR, so a setpoint step kicks it — which is why it is filtered by SPD_KDF. Defaults to zero rather than to VESC's 0.0001 because differentiating a sensorless speed estimate is noise amplification and nothing here has been tuned against a real rotor; raise it only if P and I alone leave the response sluggish.
    // @Range: 0 0.01
    // @User: Advanced
    AP_GROUPINFO("SPD_KD", 54, FOC_ESC, _p_spd_kd, 0.0f),
    // @Param: SPD_KDF
    // @DisplayName: Speed PID D filter
    // @Description: First-order low-pass coefficient applied to the D term alone (VESC s_pid_kd_filter), as a per-tick blend fraction. 1 is unfiltered, smaller is heavier filtering. Irrelevant while SPD_KD is zero.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("SPD_KDF", 55, FOC_ESC, _p_spd_kdf, 0.2f),
    // @Param: SPD_MINRPM
    // @DisplayName: Speed control release threshold
    // @Description: Below this SETPOINT the speed loop clears its integrator and releases the motor to zero torque instead of regulating (VESC s_pid_min_erpm). Speed control is meaningless at a speed the feedback cannot resolve, and without the guard the loop integrates against a setpoint it can never reach while standing still. This is also the speed at which an idle trigger in a PID control type stops braking and coasts, and the point at which a latched fault clears.
    // @Units: rpm
    // @Range: 0 20000
    // @User: Advanced
    AP_GROUPINFO("SPD_MINRPM", 56, FOC_ESC, _p_spd_minrpm, 900.0f),
    // @Param: SPD_RAMP
    // @DisplayName: Speed setpoint slew rate
    // @Description: Rate at which the speed setpoint is slewed toward the commanded speed (VESC s_pid_ramp_erpms_s). This is the acceleration AND deceleration limit of speed control: it bounds how fast the loop is allowed to ask for a change, which is what keeps the observer and PLL able to track it. Lower is gentler and slower; too low with a large step leaves the integrator saturated for the whole ramp.
    // @Units: rpm/s
    // @Range: 100 200000
    // @User: Standard
    AP_GROUPINFO("SPD_RAMP", 57, FOC_ESC, _p_spd_ramp, 25000.0f),
    // @Param: SPD_BRAKE
    // @DisplayName: Speed control braking
    // @Description: Whether the speed loop may command torque opposing rotation to hold the setpoint (VESC s_pid_allow_braking). 0 makes an over-speed coast down instead of being braked back, which matters on a bus that cannot absorb regen. The gate is on measured speed with a plus/minus 20 eRPM dead zone, so a stopped motor can still be driven either way.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("SPD_BRAKE", 58, FOC_ESC, _p_spd_brake, 1),
    // ── Sensorless open-loop start (VESC foc_sl_openloop_*) ──────────────────
    // Inactive in HALL sensor mode: the halls commutate from standstill and this
    // state machine is never entered.
    // @Param: OL_IBOOST
    // @DisplayName: Open-loop start boost current
    // @Units: A
    // @User: Advanced
    AP_GROUPINFO("OL_IBOOST", 37, FOC_ESC, _p_ol_boost, 4.0f),
    // @Param: OL_IMAX
    // @DisplayName: Open-loop iq cap
    // @Units: A
    // @User: Advanced
    AP_GROUPINFO("OL_IMAX", 38, FOC_ESC, _p_ol_imax, 15.0f),
    // @Param: OL_ERPM
    // @DisplayName: Open-loop handover speed
    // @Units: rpm
    // @User: Advanced
    AP_GROUPINFO("OL_ERPM", 39, FOC_ESC, _p_ol_erpm, 2000.0f),
    // @Param: OL_LOW
    // @DisplayName: Handover speed at zero current, as a fraction of OL_ERPM
    // @User: Advanced
    AP_GROUPINFO("OL_LOW", 40, FOC_ESC, _p_ol_low, 0.0f),
    // @Param: OL_HYST
    // @DisplayName: Time below threshold before the open-loop override fires
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("OL_HYST", 41, FOC_ESC, _p_ol_hyst, 0.1f),
    // @Param: OL_TLOCK
    // @DisplayName: Open-loop start hold-angle lock time
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("OL_TLOCK", 42, FOC_ESC, _p_ol_tlock, 0.4f),
    // @Param: OL_TRAMP
    // @DisplayName: Open-loop forced-speed ramp time
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("OL_TRAMP", 43, FOC_ESC, _p_ol_tramp, 0.5f),
    // @Param: OL_TCONST
    // @DisplayName: Open-loop forced-speed hold time after the ramp
    // @Description: OL_TLOCK + OL_TRAMP + OL_TCONST is the total forced-rotation dwell before handover is attempted, currently ~1.1 s.
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("OL_TCONST", 44, FOC_ESC, _p_ol_tconst, 0.2f),
    // @Param: V_OV
    // @DisplayName: Hard bus over-voltage trip
    // @Description: Bridge off and FAULT_OVER_VOLTAGE latched above this. The backstop behind V_MAX/V_FOLD, evaluated on the raw (unfiltered) bus sample because regen into a supply that cannot sink it climbs volts per millisecond. VESC's l_max_vin, and settable from VESC Tool in that box. Held at or above V_MAX internally.
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("V_OV", 35, FOC_ESC, _p_v_ov, 45.0f),
    // @Param: T_START
    // @DisplayName: FET temperature derate onset
    // @Units: degC
    // @User: Advanced
    AP_GROUPINFO("T_START", 22, FOC_ESC, _p_t_start, 80.0f),
    // @Param: T_MAX
    // @DisplayName: FET over-temperature hard trip
    // @Units: degC
    // @User: Advanced
    AP_GROUPINFO("T_MAX", 23, FOC_ESC, _p_t_max, 100.0f),
    // @Param: STL_RPM
    // @DisplayName: Stall speed threshold
    // @Units: rpm
    // @User: Advanced
    AP_GROUPINFO("STL_RPM", 24, FOC_ESC, _p_stall_rpm, 250.0f),
    // @Param: STL_I
    // @DisplayName: Stall current threshold
    // @Units: A
    // @User: Advanced
    AP_GROUPINFO("STL_I", 25, FOC_ESC, _p_stall_i, 2.0f),
    // @Param: STL_T
    // @DisplayName: Stall dwell before trip
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("STL_T", 26, FOC_ESC, _p_stall_t, 1.0f),
    // @Param: V_MIN
    // @DisplayName: Bus under-voltage floor
    // @Description: Bridge is gated off below this bus voltage on a single reading (no debounce); motoring current folds back over V_FOLD above it. Protects the bus-derived gate-drive rail from a current-limited supply collapsing while driving. Keep above the gate driver supply's dropout.
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("V_MIN", 27, FOC_ESC, _p_v_min, 14.0f),
    // @Param: V_UVFOLD
    // @DisplayName: Bus under-voltage foldback band
    // @Description: Motoring current scales from full at (V_MIN + this) down to zero at V_MIN. Separate from V_FOLD, which is the over-voltage regen band. Keep V_MIN + V_UVFOLD comfortably below the lowest bus voltage you actually run at, or normal supply sag will silently derate torque. VESC's equivalent (l_battery_cut_start/end) sits far below normal running voltage.
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("V_UVFOLD", 28, FOC_ESC, _p_v_uvfold, 2.0f),
    // @Param: M_OBSG
    // @DisplayName: Ortega observer gain (gamma)
    // @Description: Sensorless flux observer gain. VESC Tool's FOC tab shows and writes this as "Observer Gain (x1M)", and its detection wizard computes it as 1e3/lambda^2 after measuring the flux linkage. Scales as 1/lambda^2, so it must be re-derived whenever M_FLUX changes.
    // @User: Advanced
    AP_GROUPINFO("M_OBSG", 29, FOC_ESC, _p_obs_gain, 2.5e7f),
    // @Param: M_CKP
    // @DisplayName: Current-loop proportional gain
    // @Description: Direct current-PI Kp in volts per amp (VESC foc_current_kp). 0 = derive from the internal bandwidth and M_LS, which is the behaviour if VESC Tool has never written it.
    // @User: Advanced
    AP_GROUPINFO("M_CKP", 30, FOC_ESC, _p_cur_kp, 0.0f),
    // @Param: M_CKI
    // @DisplayName: Current-loop integral gain
    // @Description: Direct current-PI Ki in volts per amp-second (VESC foc_current_ki). 0 = derive from the internal bandwidth and M_RS.
    // @User: Advanced
    AP_GROUPINFO("M_CKI", 31, FOC_ESC, _p_cur_ki, 0.0f),
    // @Param: HL_ERPM0
    // @DisplayName: Hall-to-observer blend start
    // @Description: Commutation angle is pure hall below this electrical RPM (VESC foc_sl_erpm_start). Above HL_ERPM1 it is pure observer, linear in between. Halls cannot desync; the observer can. Keep BOTH well above your working speed until the sensorless observer has been validated on this motor.
    // @Units: rpm
    // @User: Advanced
    AP_GROUPINFO("HL_ERPM0", 32, FOC_ESC, _p_hall_erpm0, 3000.0f),
    // @Param: HL_ERPM1
    // @DisplayName: Hall-to-observer blend end
    // @Description: Commutation angle is pure observer at and above this electrical RPM (VESC foc_sl_erpm).
    // @Units: rpm
    // @User: Advanced
    AP_GROUPINFO("HL_ERPM1", 33, FOC_ESC, _p_hall_erpm1, 6000.0f),
    // @Param: HL_INTRP
    // @DisplayName: Hall angle rate-limiter floor speed
    // @Description: Floor speed for the commutation-angle slew limiter (VESC foc_hall_interp_erpm). Sets how fast the angle may move across a 60 degree sector step when the measured hall speed is near zero.
    // @Units: rpm
    // @User: Advanced
    AP_GROUPINFO("HL_INTRP", 34, FOC_ESC, _p_hall_intrp, 500.0f),

    AP_GROUPEND
};

void FOC_ESC::init(AP_HAL::UARTDriver *vesc_uart)
{
    ChibiOS::MotorControl::Config motor_cfg;
    motor_cfg.pwm_clock_hz = 20000000;
    motor_cfg.pwm_frequency_hz = 20000;
    motor_cfg.current_sample_delay_ticks = 5;
    // Bridge dead time, in real nanoseconds — the driver encodes it against the
    // timer kernel clock. This was previously "8", intended as 8 PWM counter
    // ticks (400 ns) but applied as 8 DTG steps of the 160 MHz kernel clock =
    // 50 ns. Verify the achieved value with motor_control.deadtime_ns() and on
    // a scope before running the bridge hard.
    motor_cfg.deadtime_ns = 200;
    motor_cfg.center_aligned = true;
    motor_cfg.break_input_enabled = false;
    // BDUAV 6374-170kv electrical parameters (tune on hardware via VESC Tool).
    motor_cfg.motor_Rs   = 0.055f;   // phase resistance [Ω] (Step-1 I–V slope: ≈0.055)
    motor_cfg.motor_Ls   = 80e-6f;   // phase inductance [H]
    motor_cfg.motor_flux = 4.6e-3f;  // PM flux linkage λ [Wb] (≈60/(√3·π·Kv·poles))
    motor_cfg.vbus       = 18.0f;    // DC bus [V] (fixed until bus ADC added)
    // Per-phase duty ceiling. The MP1918 high side is bootstrapped, so 100% duty
    // is not a supported state, and the low-side window it reserves is also what
    // the shunt ADC samples in. 0.80 is deliberately conservative for bring-up
    // (10 µs of low-side conduction per period at 20 kHz — ~2.8x the ADC needs);
    // it caps the modulation index at 2*(0.80-0.5) = 0.60, i.e. about a third
    // less top speed than the old 0.90. Raise toward 0.92 once the stage is
    // trusted and the sampling has been checked on a scope.
    motor_cfg.duty_max         = 0.80f;   // overridden by D_MAX below
    // Conservative limits for first bring-up — raise once verified.
    motor_cfg.current_max      = 15.0f;
    motor_cfg.overcurrent_trip = 30.0f;
    // Instant trip (no debounce, not blanked on arm) — catches arming into
    // a short within one sample. Keep well above overcurrent_trip, below
    // the EPC2305 80 A pulse rating.
    motor_cfg.overcurrent_trip_hard = 45.0f;
    // Bus-OV regen foldback: braking current scales to zero as vbus rises
    // from (vbus_max - band) to vbus_max.
    motor_cfg.vbus_max       = 40.0f;
    motor_cfg.vbus_fold_band = 3.0f;
    // Startup torque boost added on top of the commanded iq during the forced
    // I/f sequence. This is the single highest-stress routine in the firmware:
    // it holds this current on a FORCED angle, into a rotor that may not be
    // following, for openloop_lock_s + ramp_s + const_s (~1.1 s) — so even a
    // 1 A command drives this much. Five gate drivers have died on this bench;
    // start conservative and only raise it if the rotor demonstrably fails to
    // spin up (watch `lag` shrinking as back-EMF appears). Was 10 A.
    motor_cfg.openloop_current = 4.0f;   // overridden by OL_IBOOST below
    // Max braking/regen motor current [A]. Deliberately low: braking energy
    // returns to the bus, a bench PSU can't sink it, and there is no bus-OV
    // clamp yet. Raise once OV handling / a brake resistor exists.
    motor_cfg.regen_current_max = 5.0f;
    // Refuse a CURRENT↔SPEED hot-swap while peak phase current ≥ this [A];
    // the host must coast (command 0) between active modes.
    motor_cfg.mode_switch_current = 1.0f;
    // Startup spin-up: the 6374 rotor can't follow a 0.1 s ramp on only 3 A,
    // so it slips and the observer never sees coherent back-EMF. Give it real
    // torque and a gentle, long ramp/hold so we can confirm it actually spins
    // (watch `lag`→small as back-EMF appears) before worrying about handover.
    motor_cfg.openloop_max_q  = 15.0f;  // = current_max; overridden by OL_IMAX below
    // Capture phase: hold the vector static (current ramped in over the
    // first ~75 ms) until the rotor's settle oscillation dies, THEN
    // accelerate — otherwise capture happens mid-ramp and a bad draw
    // slips poles backward before catching (backward-run-then-jerk start).
    // All three overridden by OL_TLOCK / OL_TRAMP / OL_TCONST below.
    motor_cfg.openloop_lock_s = 0.4f;
    motor_cfg.openloop_ramp_s = 0.5f;   // was 0.1 s — gentler so the rotor can keep up
    motor_cfg.openloop_const_s = 0.2f;  // hold forced rotation long enough to observe
    // Hand over to the observer at HIGHER speed so back-EMF is large enough for
    // a clean lock. Note this is NOT the actual handover eRPM: like VESC
    // (mcpwm_foc utils_map on iq vs current_max), the effective threshold is
    //   map(|iq|+boost, 0, current_max, rpm_low_frac*openloop_erpm, openloop_erpm)
    // At boost_q=10 A of 15 A, a zero-throttle start hands over near 0.8·this;
    // full throttle at this ceiling — both well above the observer's speed floor.
    motor_cfg.openloop_erpm = 2000.0f;   // overridden by OL_ERPM below
    // Outer speed PID, straight from the SPD_* params in VESC's own units — see
    // the param table above and MotorControl::Config for why they are not amps.
    motor_cfg.speed_kp           = _p_spd_kp.get();
    motor_cfg.speed_ki           = _p_spd_ki.get();
    motor_cfg.speed_kd           = _p_spd_kd.get();
    motor_cfg.speed_kd_filter    = _p_spd_kdf.get();
    motor_cfg.speed_min_erpm     = _p_spd_minrpm.get();
    motor_cfg.speed_ramp_erpm_s  = _p_spd_ramp.get();
    motor_cfg.speed_allow_braking = _p_spd_brake.get() != 0;
    motor_cfg.current_scale = 1.0f / (phase_current_shunt_ohms *
                                       phase_current_shunt_input_attenuation *
                                       phase_current_amp_gain);
    // Bring-up: debug-voltage mode rotates the applied vector at this rate so
    // the observer has real back-EMF to lock onto (Step 2 sign/tracking check).
    motor_cfg.debug_openloop_hz = 1.0f;   // ≈60 eRPM forced rotation (pulls in from rest)
    // Torque-command slew: full-scale (current_max) reached in ~100 ms so a
    // PWM RC throttle step can't apply an instant iq jump. See MotorControl.
    motor_cfg.current_slew_a_s = 150.0f;   // [A/s]

    // ── Apply persisted config (AP_Periph storage) over the defaults ────────
    // Motor identity + limits + protections are now params (tunable over
    // DroneCAN/MAVLink, and the identity/limit subset over VESC Tool). These
    // override the compile-time defaults set above.
    motor_cfg.motor_Rs             = _p_motor_rs.get();
    motor_cfg.motor_Ls             = _p_motor_ls.get();
    motor_cfg.motor_flux           = _p_motor_flux.get();
    motor_cfg.current_max          = _p_i_max.get();
    motor_cfg.overcurrent_trip     = _p_i_oc.get();
    motor_cfg.overcurrent_trip_hard = _p_i_oc_hard.get();
    motor_cfg.regen_current_max    = _p_i_regen.get();
    motor_cfg.current_slew_a_s     = _p_i_slew.get();
    motor_cfg.current_scale        = _p_i_scale.get();
    motor_cfg.vbus_max             = _p_v_max.get();
    motor_cfg.vbus_min             = _p_v_min.get();
    motor_cfg.vbus_fold_band       = _p_v_fold.get();
    motor_cfg.vbus_ov_trip         = _p_v_ov.get();
    motor_cfg.vbus_uv_fold_band    = _p_v_uvfold.get();
    motor_cfg.observer_gain        = _p_obs_gain.get();
    motor_cfg.current_kp           = _p_cur_kp.get();
    motor_cfg.current_ki           = _p_cur_ki.get();
    motor_cfg.hall_blend_erpm_lo   = _p_hall_erpm0.get();
    motor_cfg.hall_blend_erpm_hi   = _p_hall_erpm1.get();
    motor_cfg.hall_interp_erpm     = _p_hall_intrp.get();
    motor_cfg.fet_temp_start       = _p_t_start.get();
    motor_cfg.fet_temp_max         = _p_t_max.get();
    motor_cfg.stall_erpm           = _p_stall_rpm.get();
    motor_cfg.stall_current        = _p_stall_i.get();
    motor_cfg.stall_time_s         = _p_stall_t.get();
    // Duty ceiling. Clamped here rather than trusting the param: it also arrives
    // from VESC Tool's l_max_duty box, and this one bounds a HARDWARE window
    // (bootstrap refresh + shunt sample aperture), not a preference. The upper
    // bound sits below the ~0.93 the ADC timing is documented good for, so the
    // margin left over is the bootstrap's — the side that has never been measured
    // on this board and the side whose failure kills FETs.
    motor_cfg.duty_max             = constrain_float(_p_duty_max.get(), 0.55f, 0.90f);
    motor_cfg.duty_start           = _p_duty_start.get();
    motor_cfg.max_erpm             = _p_max_erpm.get();
    motor_cfg.min_erpm             = _p_min_erpm.get();
    motor_cfg.erpm_start           = _p_erpm_start.get();
    motor_cfg.openloop_current     = _p_ol_boost.get();
    motor_cfg.openloop_max_q       = _p_ol_imax.get();
    motor_cfg.openloop_erpm        = _p_ol_erpm.get();
    motor_cfg.openloop_rpm_low_frac = _p_ol_low.get();
    motor_cfg.openloop_hyst_s      = _p_ol_hyst.get();
    motor_cfg.openloop_lock_s      = _p_ol_tlock.get();
    motor_cfg.openloop_ramp_s      = _p_ol_tramp.get();
    motor_cfg.openloop_const_s     = _p_ol_tconst.get();

    motor_cfg.sensor_mode = (_p_sensor_mode.get() == 0)
                                ? ChibiOS::MotorControl::SensorMode::SENSORLESS
                                : ChibiOS::MotorControl::SensorMode::HALL;
    // Use the stored hall table only once it has been populated by a detection
    // run (any entry >= 0); a virgin board keeps the compile-time default guess.
    bool have_hall_table = false;
    for (uint8_t k = 0; k < 8; k++) {
        if (_p_hall[k].get() >= 0) { have_hall_table = true; break; }
    }
    if (have_hall_table) {
        for (uint8_t k = 0; k < 8; k++) {
            const int16_t v = _p_hall[k].get();
            motor_cfg.hall_table_deg[k] = (v < 0) ? nanf("") : float(v);
        }
    }

    motor_control.init(motor_cfg);

    // Give the VESC link the param-derived values it can't read from
    // MotorControl (for the GET_MCCONF read-out) and register the write-back
    // sink for VESC Tool's "Write Motor Configuration".
    ChibiOS::VescTelemetry::ConfSnapshot snap;
    snap.poles           = uint8_t(_p_motor_poles.get() * 2);  // count = 2×pairs
    snap.max_vin          = _p_v_ov.get();                       // hard OV trip
    snap.regen_cut_end    = _p_v_max.get();                      // foldback ceiling
    snap.regen_cut_start  = _p_v_max.get() - _p_v_fold.get();    // foldback onset
    snap.temp_fet_start  = _p_t_start.get();
    snap.temp_fet_end    = _p_t_max.get();
    snap.abs_current_max = _p_i_oc_hard.get();
    snap.observer_gain   = _p_obs_gain.get();
    snap.ppm_ctrl_type   = uint8_t(_p_ppm_ctrl.get());
    snap.max_erpm_for_dir = _p_dir_erpm.get();
    snap.pid_max_erpm     = _p_pid_erpm.get();
    // The speed PID is seeded here for the window before the first update()
    // tick; set_speed_conf() keeps it live from then on.
    snap.s_pid_kp         = _p_spd_kp.get();
    snap.s_pid_ki         = _p_spd_ki.get();
    snap.s_pid_kd         = _p_spd_kd.get();
    snap.s_pid_kd_filter  = _p_spd_kdf.get();
    snap.s_pid_min_erpm   = _p_spd_minrpm.get();
    snap.s_pid_ramp_erpms_s = _p_spd_ramp.get();
    snap.s_pid_allow_braking = _p_spd_brake.get() != 0;
    vesc_telem.set_conf_snapshot(snap);
    vesc_telem.set_mcconf_sink(this, &FOC_ESC::mcconf_write_trampoline);
    vesc_telem.set_appconf_sink(this, &FOC_ESC::appconf_write_trampoline);
    vesc_telem.set_detect_sink(this, &FOC_ESC::detect_trampoline);

    vesc_telem.init(vesc_uart);

    // J305 PWM RC throttle input (TIM3_CH1 / PC6) — polled in read_pwm_throttle().
    ChibiOS::stm32_pwm_input_init();
}

// VESC Tool wrote a motor config (COMM_SET_MCCONF). Persist the standard fields
// that map to our params, then schedule a reboot so the controller re-inits with
// the new values (reboot-to-apply — matching how the params load at boot). Only
// the fields VESC exposes are here; the custom protections stay DroneCAN-only.
void FOC_ESC::on_mcconf_write(const ChibiOS::VescTelemetry::McconfIn &in)
{
    _p_motor_rs.set_and_save(in.motor_r);
    _p_motor_ls.set_and_save(in.motor_l);
    _p_motor_flux.set_and_save(in.motor_flux);
    if (in.poles >= 2) {
        _p_motor_poles.set_and_save(int8_t(in.poles / 2));   // count → pairs
    }
    _p_i_max.set_and_save(in.current_max);
    // l_current_min → I_REGEN. VESC stores it negative, so negate for our
    // positive magnitude. Guarded rather than unconditional: a zero or
    // positive-signed field is a config that never set it, and taking that at
    // face value would either disable braking entirely or store a nonsense
    // sign. Also refused above I_MAX — a braking cap looser than the motoring
    // cap is not a limit, and VESC Tool will happily send one.
    if (in.current_min < 0.0f && -in.current_min <= in.current_max) {
        _p_i_regen.set_and_save(-in.current_min);
    }
    _p_i_oc_hard.set_and_save(in.abs_current_max);
    _p_v_ov.set_and_save(in.max_vin);      // l_max_vin is VESC's hard OV trip
    // Regen foldback band. Guarded: VESC Tool sends the whole config, and an
    // inverted or non-positive pair would either invert the foldback (braking
    // cut at LOW bus voltage) or collapse it to a step. init() additionally
    // holds V_OV at or above V_MAX, so a config that puts the trip below the
    // foldback ceiling cannot fire the fault inside the normal band.
    if (in.regen_cut_start > 0.0f && in.regen_cut_end > in.regen_cut_start) {
        _p_v_max.set_and_save(in.regen_cut_end);
        _p_v_fold.set_and_save(in.regen_cut_end - in.regen_cut_start);
    }
    _p_t_start.set_and_save(in.temp_fet_start);
    _p_t_max.set_and_save(in.temp_fet_end);

    // foc_sensor_mode. VESC: 0 = SENSORLESS, 1 = ENCODER, 2 = HALL, 3 = HFI; we
    // implement only the first and third of those. Deliberately NOT a plain
    // "== 2 ? hall : sensorless": ENCODER/HFI are modes this firmware cannot run,
    // and silently reading either as SENSORLESS would drop a sensored motor onto
    // the observer path without the operator ever asking for it. Anything we do
    // not implement leaves the param untouched. 0xFF = field absent (see McconfIn).
    // Ortega observer gain. VESC Tool's detection derives this as 1e3/λ² and it
    // is meaningless at zero, so an absent/zero field leaves the param alone.
    if (in.observer_gain > 0.0f) {
        _p_obs_gain.set_and_save(in.observer_gain);
    }
    // Current-loop gains. Both must be positive to be believed: a zero Kp is a
    // dead current loop, and taking one without the other would pair a new
    // proportional term with a stale integral one.
    // Hall→observer blend band. Reject an inverted or non-positive pair rather
    // than storing it: lo >= hi collapses the blend to a step at an arbitrary
    // speed, which is precisely the desync hazard these params exist to control.
    if (in.sl_erpm_start > 0.0f && in.sl_erpm > in.sl_erpm_start) {
        _p_hall_erpm0.set_and_save(in.sl_erpm_start);
        _p_hall_erpm1.set_and_save(in.sl_erpm);
    }
    if (in.hall_interp_erpm > 1.0f) {
        _p_hall_intrp.set_and_save(in.hall_interp_erpm);
    }
    if (in.current_kp > 0.0f && in.current_ki > 0.0f) {
        _p_cur_kp.set_and_save(in.current_kp);
        _p_cur_ki.set_and_save(in.current_ki);
    }
    if (in.sensor_mode == 0 || in.sensor_mode == 2) {
        _p_sensor_mode.set_and_save(int8_t(in.sensor_mode == 2 ? 1 : 0));
    }
    // Duty ceiling. in.max_duty is already in D_MAX (per-phase) units — the wire
    // value is a MODULATION ceiling and VescTelemetry converts on parse, so the
    // window below is the hardware one and needs no scaling here.
    //
    // Range-checked here as well as at load, so a config carrying a stock VESC
    // l_max_duty of 0.95 (→ D_MAX 0.975) is REFUSED outright rather than
    // silently clamped to 0.90 — a stored value that does not match what is
    // running is how a limit stops meaning anything. An absent field (0 on the
    // wire → 0.5) falls below the window and is refused for free. Anything
    // inside is taken as intended.
    if (in.max_duty >= 0.55f && in.max_duty <= 0.90f) {
        _p_duty_max.set_and_save(in.max_duty);
    }
    // Duty foldback knee. VESC's own ">0.99 = disabled" sentinel is a legitimate
    // value, so the accepted window runs to 1.0 — but 0 (an absent field) and
    // anything under 0.30 are refused: a knee that low folds the current ceiling
    // away across the whole usable duty range, which reads as "no torque".
    if (in.duty_start >= 0.30f && in.duty_start <= 1.0f) {
        _p_duty_start.set_and_save(in.duty_start);
    }
    // Speed ceilings. Guarded on sign, which is what distinguishes a real setting
    // from an absent field here: a zero or negative MAX_ERPM (or a zero/positive
    // MIN_ERPM) would place the foldback knee at or through standstill and
    // hold the current ceiling at the floor forever — a drive that silently
    // refuses to turn. init() enforces the same invariant, so this guard is
    // about not STORING a value that would have to be corrected on the way in.
    if (in.max_erpm > 0.0f) {
        _p_max_erpm.set_and_save(in.max_erpm);
    }
    if (in.min_erpm < 0.0f) {
        _p_min_erpm.set_and_save(in.min_erpm);
    }
    // The knee. VESC's usable range is (0, 1]; ours stops at 0.99 (see ERPM_START),
    // so a config carrying exactly 1.0 — "hard limit, no taper" in VESC terms —
    // is REFUSED rather than quietly stored as 0.99. Same reasoning as D_MAX
    // above: a stored value that does not match what is running is worse than a
    // rejected write, and here the two differ in the direction that matters
    // (VESC's 1.0 steps the current off AT the limit; a silently-clamped 0.99
    // starts folding back 1% early, which on a 100000 default is 1000 eRPM).
    if (in.erpm_start >= 0.05f && in.erpm_start <= 0.99f) {
        _p_erpm_start.set_and_save(in.erpm_start);
    }
    // Outer speed PID. Every one of these is range-checked before it is stored,
    // not because VESC Tool is expected to send nonsense but because this is a
    // hand-decoded binary layout: if a field offset here is wrong by one, what
    // arrives is a neighbouring field reinterpreted, and a gain is exactly the
    // kind of value that turns a wrong number into current. The bounds are wide
    // enough to accept anything VESC Tool's own boxes allow and narrow enough
    // that a misparse lands outside them.
    //
    // Zero is ACCEPTED for the three gains — a zero Ki is VESC's documented way
    // to disable the integral term, and zero Kd is our default — so the guard is
    // a range test, not a nonzero test.
    if (in.s_pid_kp >= 0.0f && in.s_pid_kp <= 1.0f) {
        _p_spd_kp.set_and_save(in.s_pid_kp);
    }
    if (in.s_pid_ki >= 0.0f && in.s_pid_ki <= 1.0f) {
        _p_spd_ki.set_and_save(in.s_pid_ki);
    }
    if (in.s_pid_kd >= 0.0f && in.s_pid_kd <= 0.01f) {
        _p_spd_kd.set_and_save(in.s_pid_kd);
    }
    if (in.s_pid_kd_filter > 0.0f && in.s_pid_kd_filter <= 1.0f) {
        _p_spd_kdf.set_and_save(in.s_pid_kd_filter);
    }
    // A zero release threshold would leave the loop regulating against a
    // setpoint it cannot resolve at standstill, which is the exact failure
    // s_pid_min_erpm exists to prevent — treat it as absent rather than stored.
    if (in.s_pid_min_erpm > 0.0f && in.s_pid_min_erpm <= 50000.0f) {
        _p_spd_minrpm.set_and_save(in.s_pid_min_erpm);
    }
    // Likewise zero: VESC reads a zero ramp as "no ramp" and skips the slew
    // entirely (foc_math.c:505), which here would hand the loop an unbounded
    // setpoint step. Refuse it rather than reproduce that.
    if (in.s_pid_ramp_erpms_s > 0.0f && in.s_pid_ramp_erpms_s <= 1000000.0f) {
        _p_spd_ramp.set_and_save(in.s_pid_ramp_erpms_s);
    }
    _p_spd_brake.set_and_save(in.s_pid_allow_braking ? 1 : 0);
    // Sensorless open-loop start. Each guarded on its own: VESC Tool sends the
    // whole config, and a field the operator never touched must not overwrite a
    // value tuned here. The times are the ones that matter — lock+ramp+const is
    // how long full boost current sits on a FORCED angle, which is the highest-
    // stress routine in the firmware (five gate drivers have died on this bench).
    if (in.ol_boost_q > 0.0f && in.ol_boost_q <= _p_i_max.get()) {
        _p_ol_boost.set_and_save(in.ol_boost_q);
    }
    if (in.ol_max_q > 0.0f && in.ol_max_q <= _p_i_max.get()) {
        _p_ol_imax.set_and_save(in.ol_max_q);
    }
    if (in.ol_erpm > 0.0f) {
        _p_ol_erpm.set_and_save(in.ol_erpm);
    }
    if (in.ol_rpm_low >= 0.0f && in.ol_rpm_low <= 1.0f) {
        _p_ol_low.set_and_save(in.ol_rpm_low);
    }
    if (in.ol_hyst >= 0.0f) {
        _p_ol_hyst.set_and_save(in.ol_hyst);
    }
    // Cap the forced-rotation phases: a config asking to hold boost current on a
    // stationary forced angle for tens of seconds is a motor-cooking command,
    // and there is no motor temperature sensor to catch it.
    if (in.ol_t_lock >= 0.0f && in.ol_t_lock <= 5.0f) {
        _p_ol_tlock.set_and_save(in.ol_t_lock);
    }
    if (in.ol_t_ramp > 0.0f && in.ol_t_ramp <= 5.0f) {
        _p_ol_tramp.set_and_save(in.ol_t_ramp);
    }
    if (in.ol_t_const > 0.0f && in.ol_t_const <= 5.0f) {
        _p_ol_tconst.set_and_save(in.ol_t_const);
    }

    // set_and_save() only QUEUES the write for the background IO thread
    // (AP_Param::save_queue). The deferred reboot below fires 400 ms later,
    // which on this G431's flash-emulated EEPROM is not long enough to drain
    // nine of them — a single page erase is tens of ms. The reboot then threw
    // every write away, so NO VESC-Tool config write ever persisted: the board
    // kept reporting param defaults after a confirmed write-and-reboot cycle.
    //
    // Coast first: flush() blocks this (the periph main) thread for as long as
    // the queue takes, up to 2 s, and the motor must not be left under command
    // while the loop that feeds the throttle arbiter is stalled.
    motor_control.set_current(0.0f);
    AP_Param::flush();               // blocks until saved; uses expect_delay_ms

    _reboot_ms = AP_HAL::millis();   // deferred reboot (see update())
}

// Sink for VESC Tool's "Write App Configuration". Only the PPM control type, the
// direction-switch ceiling and the PID-mode speed scale are backed by params
// here; the rest of the blob describes apps this firmware does not implement and
// is discarded upstream.
void FOC_ESC::on_appconf_write(const ChibiOS::VescTelemetry::AppconfIn &in)
{
    // An unimplemented control type is REFUSED, not clamped to the nearest thing
    // we do support. Accepting it would leave VESC Tool reporting a successful
    // write of, say, "Current Smart Reverse" while the ESC quietly kept braking
    // like NOREV_BRAKE — a throttle that does not do what the box says is worse
    // than a write that visibly did not take. The read-back after the reboot
    // shows the unchanged value, which is the honest signal.
    bool accepted = false;
    if (in.ppm_ctrl_type == PPM_CTRL_CURRENT ||
        in.ppm_ctrl_type == PPM_CTRL_CURRENT_NOREV_BRAKE ||
        in.ppm_ctrl_type == PPM_CTRL_CURRENT_BRAKE_REV_HYST ||
        ppm_ctrl_is_pid(in.ppm_ctrl_type)) {
        _p_ppm_ctrl.set_and_save(float(in.ppm_ctrl_type));
        accepted = true;
        vesc_telem.note_appconf_accepted();
    }
    // Only meaningful to the PID types, but stored whatever the type is: VESC
    // Tool sends the whole page every time, and refusing the field unless the
    // type happened to be set in the same write would make the box on screen
    // silently not stick when it is edited on its own.
    if (in.pid_max_erpm > 0.0f) {
        _p_pid_erpm.set_and_save(in.pid_max_erpm);
        accepted = true;
    }
    // Zero would forbid the gesture at every speed, which is what control type 3
    // is for — treat it as an absent field rather than a setting.
    if (in.max_erpm_for_dir > 0.0f) {
        _p_dir_erpm.set_and_save(in.max_erpm_for_dir);
        accepted = true;
    }

    // Only reboot if something actually changed. A write that was refused in
    // full has nothing to apply, and rebooting anyway drops the link for no
    // reason — which also destroys the evidence, since the 'diag' counters that
    // say WHY it was refused live in RAM. Staying up leaves them readable.
    if (!accepted) {
        return;
    }

    motor_control.set_current(0.0f);
    AP_Param::flush();

    _reboot_ms = AP_HAL::millis();   // deferred reboot (see update())
}

// ── Motor-parameter detection ───────────────────────────────────────────────
//
// Answers VESC Tool's "Measure R/L", "Measure λ" and the motor wizard. The
// FLUX measurement follows conf_general_measure_flux_linkage_openloop():
// lock the rotor with d-axis current, ramp the commanded speed until the duty
// reaches the requested value, then average and solve
//     λ = (|v| − R·|i|)/ω_e − |i|·L
//
// R and L do NOT use VESC's mcpwm_foc_measure_res_ind() (voltage-step injection
// at the PWM level). They use this firmware's own two measurements, which are
// already validated on this hardware and reuse machinery that exists:
//   R  — DC d-axis injection at zero speed, R = vd/id. Runs through the normal
//        modulation path, so dead-time compensation applies and the reading is
//        not inflated by the dead band the way an uncompensated one is.
//   L  — the tone sweep. |Z| = V_tone/(1.5·Ipk) at each frequency (the 1.5
//        removes the B‖C parallel path), then a least-squares fit of
//        |Z|² = R² + ω²L². The SLOPE gives L and is insensitive to the fixed
//        voltage offset that dead time contributes, which is why L is taken
//        from the sweep and R is not.
// Different method, same quantity, reported through the standard command so
// VESC Tool is none the wiser.
namespace {
// Tone frequencies for the inductance sweep [Hz]. Above ~500 Hz so ωL is a
// meaningful share of |Z| (at ~9 µH, ωL at 500 Hz is only ~28 mΩ against ~50 mΩ
// of R), and at most 4 kHz because a 20 kHz PWM samples a 4 kHz tone just five
// times per cycle. Matches the bench sweep in bench_debug.py.
constexpr float    DET_L_FREQS[4]  = { 800.0f, 1500.0f, 2500.0f, 4000.0f };
constexpr float    DET_L_AMP       = 0.06f;   // modulation, capped by debug_max_modulation
constexpr uint32_t DET_L_TONE_MS   = 300;     // tone length per point
constexpr uint32_t DET_L_GAP_MS    = 120;     // silence between points
constexpr uint32_t DET_R_LOCK_MS   = 400;     // current ramp-in + settle before averaging
constexpr uint32_t DET_R_MEAS_MS   = 400;
// VESC's resistance search starts at 2 A and steps x1.5 until the resistive
// drop reaches ~1 V (mcpwm_foc_measure_res_ind), capped at l_current_max/2.
constexpr float    DET_R_START_A   = 2.0f;
constexpr uint32_t DET_F_LOCK_MS   = 500;     // VESC ramps in over 200 ms then dwells
constexpr uint32_t DET_F_STILL_MS  = 600;     // standstill duty average (VESC: 1000 ms)
constexpr uint32_t DET_F_RAMP_MAX_MS = 15000; // VESC max_time
constexpr uint32_t DET_F_SETTLE_MS = 1000;
constexpr uint32_t DET_F_MEAS_MS   = 1000;
constexpr float    DET_F_ERPM_MAX  = 12000.0f;// VESC stops ramping here
}

void FOC_ESC::on_detect_request(const ChibiOS::VescTelemetry::DetectReq &req)
{
    using DetectKind = ChibiOS::VescTelemetry::DetectKind;
    if (_det_state != DetState::IDLE) {
        return;   // VESC discards a blocking command while another is running
    }
    if (!motor_control.is_initialized() || motor_control.get_fault() != 0) {
        // Refuse rather than clear a latched trip — detection drives real
        // current, so the operator must release the fault first. Reply with the
        // failure value so VESC Tool reports an error instead of hanging.
        switch (req.kind) {
        case DetectKind::R_L:           vesc_telem.send_detect_r_l(0, 0, 0); break;
        case DetectKind::FLUX_OPENLOOP: vesc_telem.send_detect_flux(0);      break;
        case DetectKind::APPLY_ALL_FOC: vesc_telem.send_detect_apply_all(0); break;
        default: break;
        }
        return;
    }
    _det_req       = req;
    _det_r         = 0.0f;
    _det_l         = 0.0f;
    _det_freq_idx  = 0;
    _det_r_try     = DET_R_START_A;
    _det_acc_a     = _det_acc_b = 0.0f;
    _det_ticks     = 0;
    _det_erpm      = 0.0f;
    _det_duty_max  = 0.0f;
    _det_duty_still = 0.0f;
    _det_ms        = AP_HAL::millis();
    // APPLY_ALL_FOC is R/L followed by flux, so both entry points start at R.
    // A bare FLUX request skips straight to the spin and uses the R and L the
    // host supplied (or, if it supplied none, the configured values).
    _det_state = (req.kind == DetectKind::FLUX_OPENLOOP) ? DetState::FLUX_LOCK
                                                         : DetState::RES_LOCK;
    if (_det_state == DetState::FLUX_LOCK) {
        _det_r = (req.resistance > 0.0f) ? req.resistance : _p_motor_rs.get();
        _det_l = (req.inductance > 0.0f) ? req.inductance : _p_motor_ls.get();
    }
}

// Send the reply for whichever command is running and stand the motor down.
void FOC_ESC::detect_finish(float r, float l, float linkage, bool ok)
{
    using DetectKind = ChibiOS::VescTelemetry::DetectKind;
    motor_control.set_current(0.0f);
    motor_control.stop();
    switch (_det_req.kind) {
    case DetectKind::R_L:
        vesc_telem.send_detect_r_l(ok ? r : 0.0f, ok ? (l * 1e6f) : 0.0f, 0.0f);
        break;
    case DetectKind::FLUX_OPENLOOP:
        // VESC passes negative sentinels straight through as the linkage value;
        // preserve that so VESC Tool can show its own diagnostic text.
        vesc_telem.send_detect_flux(linkage);
        break;
    case DetectKind::APPLY_ALL_FOC:
        if (ok && linkage > 0.0f) {
            _p_motor_rs.set_and_save(r);
            _p_motor_ls.set_and_save(l);
            _p_motor_flux.set_and_save(linkage);
            // Observer gain scales as 1/λ², so a new flux linkage invalidates the
            // old gain. Derive it the way VESC Tool's wizard does (1e3/λ²) rather
            // than leaving a gain tuned for a different machine in place. Note
            // VESC's own firmware uses 0.5e3/λ² in
            // conf_general_measure_flux_linkage_openloop() — the tool's number is
            // 2× the firmware's, and we follow the TOOL so what the FOC tab shows
            // after a detection is what the board actually runs.
            _p_obs_gain.set_and_save(1.0e3f / (linkage * linkage));
            // Re-derive the current-loop gains from the freshly measured R and L,
            // mirroring VESC conf_general_calc_values(): kp = L·bw, ki = R·bw.
            // Without this, gains previously written by VESC Tool would survive as
            // explicit overrides tuned for the OLD motor parameters — the "0 =
            // derive" fallback only protects a board that has never had them set.
            // bw is our configured current-loop bandwidth (motor_cfg.current_bw_rad).
            constexpr float CUR_BW_RAD = 1000.0f;
            _p_cur_kp.set_and_save(l * CUR_BW_RAD);
            _p_cur_ki.set_and_save(r * CUR_BW_RAD);
            AP_Param::flush();
            vesc_telem.send_detect_apply_all(1);
            _reboot_ms = AP_HAL::millis();   // reboot-to-apply, same as MCCONF write
        } else {
            vesc_telem.send_detect_apply_all(0);
        }
        break;
    default:
        break;
    }
    _det_state = DetState::IDLE;
}

// Report the reason on the terminal, then finish as a failure. Every abort path
// goes through here so a failed detection always leaves a trace.
void FOC_ESC::detect_abort(const char *why, float linkage)
{
    char line[96];
    hal.util->snprintf(line, sizeof(line), "detect FAILED: %s", why);
    vesc_telem.send_print(line);
    // The numbers behind every decision, in one line — which stage it died in
    // and what the ramp had reached. Without these the reason string alone can't
    // separate "never got moving" from "got moving then lost it".
    hal.util->snprintf(line, sizeof(line),
                       "  stage=%u erpm=%.0f duty=%.3f dmax=%.3f dstill=%.3f R=%.4f L=%.2fuH",
                       unsigned(_det_state), double(_det_erpm),
                       double(motor_control.get_duty_vesc()), double(_det_duty_max),
                       double(_det_duty_still), double(_det_r), double(_det_l * 1e6f));
    vesc_telem.send_print(line);
    detect_finish(0.0f, 0.0f, linkage, false);
}

bool FOC_ESC::update_detect(uint32_t now_ms)
{
    using DetectKind = ChibiOS::VescTelemetry::DetectKind;
    if (_det_state == DetState::IDLE) {
        return false;
    }
    // Any trip during a measurement aborts it. The ISR has already gated the
    // bridge; all we do is report the failure so the tool does not hang.
    if (motor_control.get_fault() != 0) {
        char line[64];
        hal.util->snprintf(line, sizeof(line), "fault %u tripped mid-detect",
                           unsigned(motor_control.get_fault()));
        detect_abort(line);
        return false;
    }
    const uint32_t dt_ms = now_ms - _det_ms;
    float vd = 0.0f, vq = 0.0f, id = 0.0f, iq = 0.0f;
    motor_control.get_vdq(vd, vq);
    motor_control.get_idq(id, iq);

    switch (_det_state) {
    // ── Resistance: DC d-axis injection, rotor locked ───────────────────────
    case DetState::RES_LOCK:
        motor_control.set_openloop_current(_det_r_try, 0.0f);
        if (dt_ms >= DET_R_LOCK_MS) {
            _det_acc_a = _det_acc_b = 0.0f;
            _det_ticks = 0;
            _det_state = DetState::RES_MEAS;
            _det_ms    = now_ms;
        }
        break;

    case DetState::RES_MEAS:
        motor_control.set_openloop_current(_det_r_try, 0.0f);
        // VESC averages the MAGNITUDES |v| and |i| (mcpwm_foc.c:4131 accumulates
        // NORM2(vd,vq) and NORM2(id,iq)) and takes R = |v|/|i|, rather than the
        // d-axis ratio. Same thing at DC on a locked rotor, but it does not
        // assume the whole response landed on the axis we drove.
        _det_acc_a += sqrtf(vd * vd + vq * vq);
        _det_acc_b += sqrtf(id * id + iq * iq);
        _det_ticks++;
        if (dt_ms >= DET_R_MEAS_MS) {
            const float i_avg = (_det_ticks > 0) ? (_det_acc_b / float(_det_ticks)) : 0.0f;
            const float v_avg = (_det_ticks > 0) ? (_det_acc_a / float(_det_ticks)) : 0.0f;
            if (i_avg < 0.5f) {
                detect_abort("no current flowed during R injection");
                return false;
            }
            const float r_try = v_avg / i_avg;
            // THE point of VESC's search (mcpwm_foc_measure_res_ind): keep raising
            // the current, ×1.5 each time from 2 A, until `i > 1/r` — i.e. until
            // the resistive drop reaches ~1 V. Dead-time error is a roughly fixed
            // voltage (~0.1 V here), so measuring at a drop of ~1 V pushes it down
            // to a few percent, where at 4 A and 20 mΩ it was the same order as
            // the entire signal. This is why the first implementation read a third
            // of the real value.
            const float i_cap = _p_i_max.get() * 0.5f;   // VESC: l_current_max / 2
            if (r_try > 0.0f && _det_r_try > (1.0f / r_try)) {
                _det_r = r_try;                 // drop is big enough — accept it
            } else if ((_det_r_try * 1.5f) < i_cap) {
                _det_r_try *= 1.5f;             // step up and repeat
                _det_r      = r_try;            // keep the best so far
                _det_state  = DetState::RES_LOCK;
                _det_ms     = now_ms;
                break;
            } else {
                // Ran out of current headroom. VESC falls back to l_current_max/2
                // for one last measurement; we take the highest we reached, which
                // is the same reading without another ramp.
                _det_r = r_try;
            }
            motor_control.set_current(0.0f);
            motor_control.stop();
            _det_freq_idx = 0;
            _det_state    = DetState::IND_GAP;
            _det_ms       = now_ms;
        }
        break;

    // ── Inductance: tone sweep, |Z|² = R² + ω²L² ────────────────────────────
    case DetState::IND_GAP:
        if (dt_ms >= DET_L_GAP_MS) {
            motor_control.play_tone(DET_L_FREQS[_det_freq_idx], DET_L_AMP,
                                    uint16_t(DET_L_TONE_MS));
            _det_state = DetState::IND_TONE;
            _det_ms    = now_ms;
        }
        break;

    case DetState::IND_TONE:
        if (dt_ms >= DET_L_TONE_MS) {
            const float ipk  = motor_control.get_tone_ipk();
            const float vbus = motor_control.get_vbus();
            // Commanded phase-A amplitude. m_alpha = v·2/vbus, so v = m·vbus/2.
            const float v_cmd = DET_L_AMP * vbus * 0.5f;
            const float w     = 2.0f * float(M_PI) * DET_L_FREQS[_det_freq_idx];
            if (ipk > 0.05f) {
                const float z = v_cmd / (1.5f * ipk);
                _det_zz[_det_freq_idx] = z * z;
                _det_ww[_det_freq_idx] = w * w;
            } else {
                _det_zz[_det_freq_idx] = 0.0f;   // no response — excluded by the fit
                _det_ww[_det_freq_idx] = w * w;
            }
            _det_freq_idx++;
            if (_det_freq_idx >= 4) {
                // Least squares on (ω², |Z|²): slope = L², intercept = R².
                float sx = 0, sy = 0, sxx = 0, sxy = 0, n = 0;
                for (uint8_t k = 0; k < 4; k++) {
                    if (_det_zz[k] <= 0.0f) {
                        continue;
                    }
                    sx += _det_ww[k];  sy += _det_zz[k];
                    sxx += _det_ww[k] * _det_ww[k];
                    sxy += _det_ww[k] * _det_zz[k];
                    n += 1.0f;
                }
                const float den = n * sxx - sx * sx;
                const float slope = (n >= 2.0f && fabsf(den) > 1e-12f)
                                        ? ((n * sxy - sx * sy) / den) : 0.0f;
                _det_l = (slope > 0.0f) ? sqrtf(slope) : 0.0f;
                if (_det_l <= 0.0f) {
                    detect_abort("inductance fit gave no positive slope");
                    return false;
                }
                if (_det_req.kind == DetectKind::R_L) {
                    detect_finish(_det_r, _det_l, 0.0f, true);
                    return false;
                }
                // APPLY_ALL_FOC: carry R and L into the flux measurement.
                _det_req.resistance = _det_r;
                _det_req.inductance = _det_l;
                if (_det_req.current <= 0.0f) {
                    // Size the injection from the requested power loss budget:
                    // P = 1.5·R·I² for a three-phase machine.
                    const float pl = (_det_req.max_power_loss > 0.0f)
                                         ? _det_req.max_power_loss : 20.0f;
                    _det_req.current = sqrtf(pl / (1.5f * _det_r));
                }
                if (_det_req.duty <= 0.0f)         { _det_req.duty = 0.3f; }
                if (_det_req.erpm_per_sec <= 0.0f) { _det_req.erpm_per_sec = 1500.0f; }
                _det_state = DetState::FLUX_LOCK;
                _det_ms    = now_ms;
            } else {
                _det_state = DetState::IND_GAP;
                _det_ms    = now_ms;
            }
        }
        break;

    // ── Flux linkage: VESC conf_general_measure_flux_linkage_openloop() ─────
    case DetState::FLUX_LOCK: {
        // Ramp the current in rather than stepping it: a step into an unknown
        // rotor angle maximally excites the swing into the I/f well, which is
        // the backward-run-then-jerk start. VESC ramps over 200 ms.
        const float ramp = (dt_ms < 200) ? (float(dt_ms) / 200.0f) : 1.0f;
        motor_control.set_openloop_current(_det_req.current * ramp, 0.0f);
        if (dt_ms >= DET_F_LOCK_MS) {
            _det_acc_a = 0.0f;
            _det_ticks = 0;
            _det_state = DetState::FLUX_STILL;
            _det_ms    = now_ms;
        }
        break;
    }

    case DetState::FLUX_STILL:
        motor_control.set_openloop_current(_det_req.current, 0.0f);
        _det_acc_a += motor_control.get_duty_vesc();
        _det_ticks++;
        if (dt_ms >= DET_F_STILL_MS) {
            _det_duty_still = (_det_ticks > 0) ? (_det_acc_a / float(_det_ticks)) : 0.0f;
            _det_erpm     = 0.0f;
            _det_duty_max = 0.0f;
            _det_state    = DetState::FLUX_RAMP;
            _det_ms       = now_ms;
        }
        break;

    case DetState::FLUX_RAMP: {
        // Speed ramp at the requested eRPM/s. VESC accumulates a step every 1 ms;
        // deriving it from elapsed phase time instead is exact, independent of the
        // main-loop rate, and carries no state that could survive an aborted run
        // into the next one.
        _det_erpm = _det_req.erpm_per_sec * float(dt_ms) * 1e-3f;
        motor_control.set_openloop_current(_det_req.current, _det_erpm);

        const float duty_now = motor_control.get_duty_vesc();
        if (duty_now > _det_duty_max) {
            _det_duty_max = duty_now;
        }
        // VESC's three abort sentinels, reported as the linkage value so VESC
        // Tool shows its own explanation for each.
        if (dt_ms >= DET_F_RAMP_MAX_MS) {
            detect_abort("ramp timed out before reaching target duty", -1.0f);
            return false;
        }
        if (dt_ms > 4000 && duty_now < (_det_duty_max * 0.7f)) {
            detect_abort("duty collapsed - rotor lost sync with the forced angle", -2.0f);
            return false;
        }
        if (dt_ms > 4000 && _det_req.duty < (_det_duty_still * 1.1f)) {
            detect_abort("target duty is below the standstill duty", -3.0f);
            return false;
        }
        if (duty_now >= _det_req.duty || _det_erpm >= DET_F_ERPM_MAX) {
            _det_state = DetState::FLUX_SETTLE;
            _det_ms    = now_ms;
        }
        break;
    }

    case DetState::FLUX_SETTLE:
        motor_control.set_openloop_current(_det_req.current, _det_erpm);
        if (dt_ms >= DET_F_SETTLE_MS) {
            _det_acc_a = _det_acc_b = 0.0f;
            _det_ticks = 0;
            _det_state = DetState::FLUX_MEAS;
            _det_ms    = now_ms;
        }
        break;

    case DetState::FLUX_MEAS:
        motor_control.set_openloop_current(_det_req.current, _det_erpm);
        _det_acc_a += sqrtf(vd * vd + vq * vq);   // |v|
        _det_acc_b += sqrtf(id * id + iq * iq);   // |i|
        _det_ticks++;
        if (dt_ms >= DET_F_MEAS_MS) {
            const float v_mag = (_det_ticks > 0) ? (_det_acc_a / float(_det_ticks)) : 0.0f;
            const float i_mag = (_det_ticks > 0) ? (_det_acc_b / float(_det_ticks)) : 0.0f;
            // ω from the COMMANDED speed — in forced-angle injection that is the
            // rotor speed by construction, and it carries no estimator noise.
            const float rad_s = _det_erpm * (2.0f * float(M_PI) / 60.0f);
            float linkage = 0.0f;
            if (rad_s > 1.0f) {
                linkage = (v_mag - _det_r * i_mag) / rad_s - i_mag * _det_l;
            }
            if (!(linkage > 0.0f)) {
                // Worth the extra line: this one fails on the ARITHMETIC, not on
                // the spin, so the ramp diagnostics above look perfectly healthy.
                char line[96];
                hal.util->snprintf(line, sizeof(line),
                                   "linkage <= 0 (|v|=%.3f |i|=%.2f rad/s=%.0f)",
                                   double(v_mag), double(i_mag), double(rad_s));
                detect_abort(line);
                return false;
            }
            detect_finish(_det_r, _det_l, linkage, true);
            return false;
        }
        break;

    case DetState::DONE_STOP:
    case DetState::IDLE:
        _det_state = DetState::IDLE;
        return false;
    }
    return true;   // still ours — the arbiter stands off
}

// Poll the TIM3 capture and, on a valid in-range frame, refresh the PWM source.
// Enforces a boot-low arming gate: the throttle must be seen fully closed once
// before it may command torque, so a power-up (or reconnect) at high stick can't
// spin the motor. An out-of-range pulse re-arms the gate.
void FOC_ESC::read_pwm_throttle(uint32_t now_ms)
{
    uint16_t pulse_us, period_us;
    if (!ChibiOS::stm32_pwm_input_read(&pulse_us, &period_us)) {
        return;   // no fresh frame — leave _pwm_ms to age out into coast
    }
    // Plausibility: pulse in the RC band and a sane ~50–500 Hz frame period.
    if (pulse_us < THR_PWM_MIN_US - THR_PWM_RANGE_TOL_US ||
        pulse_us > THR_PWM_MAX_US + THR_PWM_RANGE_TOL_US ||
        period_us < 2000 || period_us > 25000) {
        _pwm_armed = false;   // bad signal → require a fresh idle before driving
        _pwm_amps  = 0.0f;
        _pwm_erpm  = 0.0f;
        _pwm_out   = PwmOut::CURRENT;
        return;               // and let the source go stale → coast
    }
    // Frame is valid → the PWM source is present (prevents coast even at neutral).
    _pwm_ms = now_ms;

    constexpr uint16_t fwd_break = THR_PWM_CTR_US + THR_PWM_DEADBAND_US;
    constexpr uint16_t brk_break = THR_PWM_CTR_US - THR_PWM_DEADBAND_US;
    // One-sided types idle at minimum pulse, two-sided ones at centre.
    constexpr uint16_t low_break = THR_PWM_MIN_US + THR_PWM_DEADBAND_US;

    const uint8_t ctrl  = uint8_t(_p_ppm_ctrl.get());
    if (ctrl != _pwm_ctrl_seen) {
        _pwm_ctrl_seen = ctrl;
        _pwm_armed     = false;   // re-arm at the new type's idle position
    }
    const bool    norev = ppm_ctrl_is_norev(ctrl);
    const bool    idle  = norev ? (pulse_us <= low_break)
                                : (pulse_us >= brk_break && pulse_us <= fwd_break);

    if (!_pwm_armed) {
        // Must be seen at idle before it can drive. A boot or reconnect with the
        // trigger held stays coasted until it is released. Which stick position
        // counts as idle depends on the control type — for a one-sided type,
        // centre is half throttle, so arming there would be arming at speed.
        if (idle) {
            _pwm_armed = true;
        }
        _pwm_amps  = 0.0f;    // hold coast until armed through idle
        _pwm_erpm  = 0.0f;
        _pwm_out   = PwmOut::CURRENT;
        // Drop any half-finished reverse gesture. VESC keeps its statics across
        // a signal loss; we do not, deliberately — coming back from a dropout or
        // a boot with the gesture already half-complete would let the first
        // back-stick go straight to reverse, and re-doing it costs one brake.
        _ppm_force_brake = true;
        _ppm_did_idle    = 0;
        return;
    }

    // Map to VESC's servo_val, with the deadband removed so the usable travel
    // starts at zero output. Two-sided types (app_ppm.c:155) map about centre
    // onto [-1, +1]; one-sided types (app_ppm.c:144-151) map the whole travel
    // onto [0, +1] and ignore the centre pulse entirely.
    float val;
    if (norev) {
        val = (pulse_us > low_break)
                  ? float(pulse_us - low_break) / float(THR_PWM_MAX_US - low_break)
                  : 0.0f;
        val = constrain_float(val, 0.0f, 1.0f);
    } else {
        if (pulse_us > fwd_break) {
            val = float(pulse_us - fwd_break) / float(THR_PWM_MAX_US - fwd_break);
        } else if (pulse_us < brk_break) {
            val = -float(brk_break - pulse_us) / float(brk_break - THR_PWM_MIN_US);
        } else {
            val = 0.0f;       // neutral band
        }
        val = constrain_float(val, -1.0f, 1.0f);
    }

    const float erpm  = motor_control.get_erpm();
    const float i_mot = motor_control.current_limit();
    const float i_brk = motor_control.regen_limit();

    if (ppm_ctrl_is_pid(ctrl)) {
        // VESC PPM_CTRL_TYPE_PID / _PID_NOREV (app_ppm.c:341-350). Both hand the
        // same thing to the motor — `set_pid_speed(servo_val * pid_max_erpm)` —
        // and differ only in the mapping above, which is why they share a branch
        // here as they do there.
        //
        // Everything that makes this feel like a throttle rather than a step
        // command lives in the speed loop, not here: the setpoint slew (SPD_RAMP)
        // bounds acceleration and deceleration, and SPD_MINRPM decides when a
        // released trigger stops braking and coasts. So an idle stick is passed
        // through as a real setpoint of zero rather than being turned into a
        // coast at this level — see MotorControl::set_rpm's zero_is_stop.
        _pwm_out  = PwmOut::SPEED;
        _pwm_amps = 0.0f;
        _pwm_erpm = val * _p_pid_erpm.get();
        return;
    }

    if (ctrl == PPM_CTRL_CURRENT) {
        // VESC PPM_CTRL_TYPE_CURRENT (app_ppm.c:300-307). Seamless bidirectional
        // torque: one SIGNED current command, never a brake command. Back-stick
        // while rolling forward is negative current, which already is regenerative
        // braking; when the eRPM crosses zero that same unchanged command simply
        // becomes reverse acceleration. No gesture, no state, no neutral visit —
        // which is precisely the property CURRENT_BRAKE_REV_HYST exists to deny,
        // so a back-stick held through the stop WILL launch backwards here.
        //
        // Which ceiling applies is decided by which way the energy is flowing, not
        // by the sign of the stick: torque that agrees with the direction of travel
        // is motoring and draws on I_MAX, torque that opposes it is pushing charge
        // back into the bus and draws on I_REGEN.
        //
        // VESC tests the raw sign (`rpm_now >= 0.0`), which is safe there only
        // because stock configs set l_current_min = -l_current_max: both branches
        // then yield the same ceiling, so the flip is invisible. Ours are
        // deliberately asymmetric, so a bare sign compare would let PLL noise at a
        // standstill chatter the ceiling by that whole ratio. Reusing the
        // THR_REV_ERPM deadband treats "stopped" as agreeing with whatever was
        // asked, which is also the physically honest answer — with no rotation
        // there is nothing to regenerate from, so a standstill command is always
        // pure motoring.
        const bool opposing = (val > 0.0f && erpm < -THR_REV_ERPM) ||
                              (val < 0.0f && erpm >  THR_REV_ERPM);
        _pwm_out  = PwmOut::CURRENT;
        _pwm_amps = val * (opposing ? i_brk : i_mot);
        return;
    }

    if (ctrl == PPM_CTRL_CURRENT_BRAKE_REV_HYST) {
        // VESC PPM_CTRL_TYPE_CURRENT_BRAKE_REV_HYST (app_ppm.c:227-294).
        //
        // Reverse is deliberately NOT seamless here. Pulling back brakes; to
        // actually reverse you must brake to a stop, return the stick to centre,
        // and pull back a second time. That is the whole point of the mode: a
        // panic brake grab, which is one continuous back-stick, can never turn
        // into reverse acceleration at the moment the vehicle stops.
        //
        // Two pieces of state carry the gesture, both mirroring VESC's statics:
        // _ppm_did_idle tracks how far through it you are, and _ppm_force_brake
        // locks out the whole thing above the speed ceiling.
        const float dir_erpm = _p_dir_erpm.get();
        const float dir_hyst = dir_erpm * 0.20f;   // VESC: 20% band, app_ppm.c:68

        // Speed gate, with hysteresis so a speed sitting on the threshold cannot
        // chatter the lockout on and off.
        if (_ppm_force_brake) {
            if (erpm < dir_erpm - dir_hyst) {
                _ppm_force_brake = false;
                _ppm_did_idle    = 0;
            }
        } else if (erpm > dir_erpm + dir_hyst) {
            _ppm_force_brake = true;
            _ppm_did_idle    = 0;
        }

        bool brake = false;
        float amps;

        if (val >= 0.0f) {
            // Exact compare against zero is intentional, not a float slip: the
            // deadband above assigns literal 0.0f inside the neutral band, so
            // this is "stick is in the neutral band", not "stick is near zero".
            if (val == 0.0f) {
                // Returned to idle after a brake — the second half of the gesture.
                if (_ppm_did_idle == 1 && !_ppm_force_brake) {
                    _ppm_did_idle = 2;
                }
            } else if (erpm > -dir_erpm) {
                // Forward command while not already reversing fast: whatever
                // gesture was in progress is abandoned.
                _ppm_did_idle = 0;
            }
            amps = val * ((erpm >= 0.0f) ? i_mot : i_brk);
        } else {
            if (_ppm_force_brake) {
                brake = true;                       // too fast to switch direction
            } else if (erpm > -dir_erpm) {
                if (_ppm_did_idle != 2) {           // gesture not complete → brake
                    _ppm_did_idle = 1;
                    brake = true;
                }
            } else if (_ppm_did_idle == 1) {
                brake = true;
            } else {
                _ppm_did_idle = 2;                  // already reversing; let it run
            }
            // Reverse acceleration is bounded by the BRAKING ceiling, not the
            // motoring one — VESC's choice (app_ppm.c:286-292), and it keeps
            // reverse gentle without needing a second limit.
            amps = val * i_brk;
        }

        _pwm_out  = brake ? PwmOut::BRAKE : PwmOut::CURRENT;
        _pwm_amps = brake ? fabsf(amps) : amps;
        return;
    }

    // VESC PPM_CTRL_TYPE_CURRENT_NOREV_BRAKE (app_ppm.c:314-323), the default:
    //     current_mode_brake = servo_val < 0
    //     forward trigger AND rolling forward → servo_val * l_current_max
    //     otherwise                           → |servo_val * l_current_min|
    // The third case is the one that is easy to miss: a FORWARD trigger while
    // rolling backwards is not a brake command, it is forward torque against the
    // roll — so it stays a motoring command (positive current, no reverse), just
    // limited to the braking ceiling because that is what is decelerating the bus.
    if (val < 0.0f) {
        _pwm_out  = PwmOut::BRAKE;
        _pwm_amps = -val * i_brk;
    } else if (erpm < -THR_REV_ERPM) {
        _pwm_out  = PwmOut::CURRENT;
        _pwm_amps = val * i_brk;
    } else {
        _pwm_out  = PwmOut::CURRENT;
        _pwm_amps = val * i_mot;
    }
}

void FOC_ESC::set_can_throttle(float frac)
{
    _can_amps = constrain_float(frac, 0.0f, 1.0f) * motor_control.current_limit();
    _can_ms   = AP_HAL::millis();
}

// Torque set-points come from three sources: DroneCAN ESC RawCommand (CAN), a
// VESC-Tool COMM_SET_CURRENT over USB serial (USB), and the J305 PWM RC input
// (PWM). Priority CAN > USB > PWM; as long as one source is fresh the motor is
// driven, and it coasts only when all are stale. A VESC-Tool bench override
// (rpm / brake / duty-debug) drives MotorControl directly and, while active,
// takes exclusive control — the arbiter stands off so it isn't stomped.
// Power-on chime: play the note table through the motor windings once the
// controller has finished zero-current calibration. Returns true while the chime
// owns the motor (so the arbiter stands off); aborts immediately if a real
// command arrives or a fault trips, so it can never sound over a live throttle.
bool FOC_ESC::update_startup_chime(uint32_t now_ms, bool any_command)
{
    if (_chime_state == ChimeState::DONE) {
        return false;
    }
    if (any_command || motor_control.get_fault() != 0) {
        // Abort the chime. Never stop() on a fault — stop() is the operator-level
        // reset and would clear the latched trip we are aborting for. The ISR
        // already holds the bridge off, so just stand down.
        if (motor_control.is_beeping() && motor_control.get_fault() == 0) {
            motor_control.stop();
        }
        _chime_state = ChimeState::DONE;
        return false;
    }
    if (_chime_state == ChimeState::WAIT) {
        // Two independent readiness conditions, and the chime needs BOTH before it
        // may drive the bridge:
        //   zero_valid()  — phase-current baseline captured (INA181 ref rail up)
        //   vbus_ready()  — bus measurement has settled through its RC filter
        // They settle on very different timescales: the INA reference comes up
        // fast, the VBUS divider is 357k||10k against 100 nF (τ ≈ 1 ms). Waiting
        // only on the first one armed the bridge — and the under-voltage trip
        // with it — against a bus reading that had not arrived yet, which is what
        // killed the chime part-way through.
        if (!motor_control.zero_valid() || !motor_control.vbus_ready()) {
            // Don't wait forever: returning true holds the arbiter off, so a bus
            // that never reads ready (e.g. V_MIN set above the actual supply)
            // would silently block throttle entirely. Give up and skip the chime
            // instead — losing a beep is preferable to losing the motor.
            if (_chime_step_ms == 0) {
                _chime_step_ms = now_ms + CHIME_READY_TIMEOUT_MS;
            } else if (int32_t(now_ms - _chime_step_ms) >= 0) {
                _chime_state = ChimeState::DONE;
                return false;
            }
            return true;   // not ready — hold the arbiter off, stay silent
        }
        _chime_idx = 0;
        motor_control.play_tone(CHIME[0].freq_hz, CHIME_AMPLITUDE, CHIME[0].ms);
        _chime_step_ms = now_ms + CHIME[0].ms + CHIME_GAP_MS;
        _chime_state   = ChimeState::PLAY;
        return true;
    }
    // PLAY: let the current note + trailing gap elapse, then start the next.
    if (int32_t(now_ms - _chime_step_ms) < 0) {
        return true;
    }
    if (++_chime_idx >= CHIME_LEN) {
        _chime_state = ChimeState::DONE;   // finished → hand control to the arbiter
        return false;
    }
    motor_control.play_tone(CHIME[_chime_idx].freq_hz, CHIME_AMPLITUDE, CHIME[_chime_idx].ms);
    _chime_step_ms = now_ms + CHIME[_chime_idx].ms + CHIME_GAP_MS;
    return true;
}

void FOC_ESC::update(uint32_t now_ms)
{
    if (motor_control.is_initialized()) {
        read_pwm_throttle(now_ms);
        motor_control.update_thermal(now_ms);     // FET NTC → iq derate / over-temp trip

        const bool can_fresh = _can_ms != 0 && (now_ms - _can_ms) < THR_SOURCE_TIMEOUT_MS;
        const bool pwm_fresh = _pwm_ms != 0 && (now_ms - _pwm_ms) < THR_SOURCE_TIMEOUT_MS;
        float usb_amps;
        const bool usb_fresh = vesc_telem.usb_current(now_ms, THR_SOURCE_TIMEOUT_MS, usb_amps);
        const bool override  = vesc_telem.override_active(now_ms, THR_SOURCE_TIMEOUT_MS);

        if (motor_control.is_hall_detecting()) {
            // A hall-table detection spin owns the motor. It is self-terminating
            // (~12 s) and coasts itself when done, so there is nothing to drive
            // and nothing to time out here. This branch must come FIRST and must
            // not depend on the VESC override flag: override_active() is keyed on
            // host-link freshness, and a host that sends the one detect command
            // then waits silently for the result goes "stale" after 200 ms — the
            // arbiter would fall through to the coast branch below and stop the
            // spin a fifth of a second in, long before any hall state is recorded.
        } else if (update_detect(now_ms)) {
            // A parameter measurement owns the motor: it drives forced-angle
            // current and any throttle input mid-run would corrupt the result.
        } else if (update_startup_chime(now_ms, can_fresh || usb_fresh || pwm_fresh || override)) {
            // Power-on chime owns the motor until it finishes or is pre-empted.
        } else if (override) {
            // VESC bench override owns the motor — leave it be.
        } else if (can_fresh) {
            motor_control.set_current(_can_amps);
        } else if (usb_fresh) {
            motor_control.set_current(usb_amps);
        } else if (pwm_fresh) {
            // The trigger is the only source that can command deceleration, so it
            // is the only one that reaches set_brake_current(). Note the brake
            // releases to IDLE below ~100 eRPM (MotorControl.cpp:1226) — there is
            // no standstill hold, by design.
            //
            // It is likewise the only source that can command a SPEED: the PID
            // control types close the outer loop on the trigger. zero_is_stop is
            // false there so an idle trigger is a setpoint of zero the loop ramps
            // down to (braking as it goes, exactly as VESC does) rather than an
            // instant coast; it collapses to a release below SPD_MINRPM.
            switch (_pwm_out) {
            case PwmOut::BRAKE:
                motor_control.set_brake_current(_pwm_amps);
                break;
            case PwmOut::SPEED:
                motor_control.set_rpm(_pwm_erpm, false);
                break;
            case PwmOut::CURRENT:
                motor_control.set_current(_pwm_amps);
                break;
            }
        } else {
            motor_control.set_current(0.0f);          // all sources stale → coast
        }
    }

    // Keep the PPM half of the snapshot LIVE rather than boot-time. Two reasons:
    // VESC Tool re-reads to confirm a write and that read lands before the
    // deferred reboot, and 'diag' reports this value — so it must be what the
    // param actually holds right now, not what it held at boot. Anything that
    // fails to persist then shows up as a run_ctrl that reverts after a reset.
    vesc_telem.set_ppm_conf(uint8_t(_p_ppm_ctrl.get()), _p_dir_erpm.get(),
                            _p_pid_erpm.get());
    vesc_telem.set_speed_conf(_p_spd_kp.get(), _p_spd_ki.get(), _p_spd_kd.get(),
                              _p_spd_kdf.get(), _p_spd_minrpm.get(),
                              _p_spd_ramp.get(), _p_spd_brake.get() != 0);
    vesc_telem.set_storage_full(AP_Param::get_eeprom_full());
    vesc_telem.update();

    // Persist a freshly detected hall table so it survives reboot (triggered via
    // VESC COMM_DETECT_HALL_FOC / hall_detect.py). One-shot per detection.
    float hd[8];
    if (motor_control.take_hall_detect_result(hd)) {
        for (uint8_t k = 0; k < 8; k++) {
            int16_t v = -1;   // unused state
            if (!isnan(hd[k])) {
                int a = int(lroundf(hd[k])) % 360;
                if (a < 0) a += 360;
                v = int16_t(a);
            }
            _p_hall[k].set_and_save(v);
        }
    }

    // Deferred reboot after a VESC-Tool config write: give the COMM_SET_MCCONF
    // ack time to go out and the param storage flush to settle, then re-init the
    // controller with the new values. Coast the motor first for safety.
    if (_reboot_ms != 0 && (now_ms - _reboot_ms) > 400) {
        _reboot_ms = 0;
        motor_control.set_current(0.0f);
        motor_control.disable_outputs();
        hal.scheduler->reboot(false);
    }
}

#endif  // HAL_PERIPH_ENABLE_FOC_ESC
