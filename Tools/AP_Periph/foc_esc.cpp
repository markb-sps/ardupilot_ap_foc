#include "foc_esc.h"

#ifdef HAL_PERIPH_ENABLE_FOC_ESC

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_ChibiOS/stm32_pwm_input.h>

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
constexpr uint16_t THR_PWM_MIN_US       = 1000;  // 0% throttle pulse
constexpr uint16_t THR_PWM_MAX_US       = 2000;  // 100% throttle pulse
constexpr uint16_t THR_PWM_DEADZONE_US  = 30;    // ignore this much above min (0% band)
constexpr uint16_t THR_PWM_RANGE_TOL_US = 100;   // accept 900..2100; beyond → signal invalid
// Arming gate: the throttle must be seen at/below this (fully closed) before it
// may command torque, so a power-up/reconnect at a raised stick won't spin the
// motor until it has passed through the low end at least once.
constexpr uint16_t THR_PWM_ARM_MAX_US   = 1000;

// A source is "fresh" for this long after its last valid command — covers a
// couple of dropped 50 Hz RC frames / DroneCAN commands before it ages to coast.
constexpr uint32_t THR_SOURCE_TIMEOUT_MS = 200;

// ── Power-on chime (played through the motor windings) ──────────────────────
struct ChimeNote { uint16_t freq_hz; uint16_t ms; };
constexpr ChimeNote CHIME[] = { {1047, 120}, {1319, 120}, {1568, 200} };  // C6–E6–G6
constexpr uint8_t   CHIME_LEN       = sizeof(CHIME) / sizeof(CHIME[0]);
constexpr float     CHIME_AMPLITUDE = 0.08f;  // modulation fraction (capped in play_tone)
constexpr uint16_t  CHIME_GAP_MS    = 40;     // silence between notes
}

void FOC_ESC::init(AP_HAL::UARTDriver *vesc_uart)
{
    ChibiOS::MotorControl::Config motor_cfg;
    motor_cfg.pwm_clock_hz = 20000000;
    motor_cfg.pwm_frequency_hz = 20000;
    motor_cfg.current_sample_delay_ticks = 5;
    motor_cfg.deadtime_ticks = 8;
    motor_cfg.center_aligned = true;
    motor_cfg.break_input_enabled = false;
    // BDUAV 6374-170kv electrical parameters (tune on hardware via VESC Tool).
    motor_cfg.motor_Rs   = 0.055f;   // phase resistance [Ω] (Step-1 I–V slope: ≈0.055)
    motor_cfg.motor_Ls   = 80e-6f;   // phase inductance [H]
    motor_cfg.motor_flux = 4.6e-3f;  // PM flux linkage λ [Wb] (≈60/(√3·π·Kv·poles))
    motor_cfg.vbus       = 18.0f;    // DC bus [V] (fixed until bus ADC added)
    // Conservative limits for first bring-up — raise once verified.
    motor_cfg.current_max      = 15.0f;
    motor_cfg.overcurrent_trip = 30.0f;
    // Instant trip (no debounce, not blanked on arm) — catches arming into
    // a short within one sample. Keep well above overcurrent_trip, below
    // the EPC23102 65 A pulse rating.
    motor_cfg.overcurrent_trip_hard = 45.0f;
    // Bus-OV regen foldback: braking current scales to zero as vbus rises
    // from (vbus_max - band) to vbus_max.
    motor_cfg.vbus_max       = 40.0f;
    motor_cfg.vbus_fold_band = 3.0f;
    // Startup torque: the OL cap was leaving a third of the current budget
    // unused (easily hand-stalled) — let open loop use the full iq limit.
    motor_cfg.openloop_current = 10.0f;
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
    motor_cfg.openloop_max_q  = 15.0f;  // = current_max: full budget for startup torque
    // Capture phase: hold the vector static (current ramped in over the
    // first ~75 ms) until the rotor's settle oscillation dies, THEN
    // accelerate — otherwise capture happens mid-ramp and a bad draw
    // slips poles backward before catching (backward-run-then-jerk start).
    motor_cfg.openloop_lock_s = 0.4f;
    motor_cfg.openloop_ramp_s = 0.5f;   // was 0.1 s — gentler so the rotor can keep up
    motor_cfg.openloop_const_s = 0.2f;  // hold forced rotation long enough to observe
    // Hand over to the observer at HIGHER speed so back-EMF is large enough for
    // a clean lock. Note this is NOT the actual handover eRPM: like VESC
    // (mcpwm_foc utils_map on iq vs current_max), the effective threshold is
    //   map(|iq|+boost, 0, current_max, rpm_low_frac*openloop_erpm, openloop_erpm)
    // At boost_q=10 A of 15 A, a zero-throttle start hands over near 0.8·this;
    // full throttle at this ceiling — both well above the observer's speed floor.
    motor_cfg.openloop_erpm = 2000.0f;
    // Outer speed PI — strong enough to reject load (tune: ↑ if sluggish, ↓ if hunting).
    motor_cfg.speed_kp = 0.005f;   // [A per eRPM]
    motor_cfg.speed_ki = 0.05f;    // [A per eRPM·s]
    motor_cfg.current_scale = 1.0f / (phase_current_shunt_ohms *
                                       phase_current_shunt_input_attenuation *
                                       phase_current_amp_gain);
    // Bring-up: debug-voltage mode rotates the applied vector at this rate so
    // the observer has real back-EMF to lock onto (Step 2 sign/tracking check).
    motor_cfg.debug_openloop_hz = 1.0f;   // ≈60 eRPM forced rotation (pulls in from rest)
    // Torque-command slew: full-scale (current_max) reached in ~100 ms so a
    // PWM RC throttle step can't apply an instant iq jump. See MotorControl.
    motor_cfg.current_slew_a_s = 150.0f;   // [A/s]
    motor_control.init(motor_cfg);

    vesc_telem.init(vesc_uart);

    // J305 PWM RC throttle input (TIM3_CH1 / PC6) — polled in read_pwm_throttle().
    ChibiOS::stm32_pwm_input_init();
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
        _pwm_armed = false;   // bad signal → require a fresh low before driving
        _pwm_amps  = 0.0f;
        return;               // and let the source go stale → coast
    }
    // Frame is valid → the PWM source is present (prevents coast even at 0%).
    _pwm_ms = now_ms;
    const uint16_t low = THR_PWM_MIN_US + THR_PWM_DEADZONE_US;
    if (!_pwm_armed) {
        // Must pass through the fully-closed end (≤1000 µs) before it can drive:
        // a boot/reconnect at raised throttle stays coasted until stick is low.
        if (pulse_us <= THR_PWM_ARM_MAX_US) {
            _pwm_armed = true;
        }
        _pwm_amps = 0.0f;     // hold coast until armed through low
        return;
    }
    const float pct = constrain_float(float(pulse_us - low) /
                                      float(THR_PWM_MAX_US - low), 0.0f, 1.0f);
    _pwm_amps = pct * motor_control.current_limit();
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
        if (motor_control.is_beeping()) {
            motor_control.stop();
        }
        _chime_state = ChimeState::DONE;
        return false;
    }
    if (_chime_state == ChimeState::WAIT) {
        if (!motor_control.zero_valid()) {
            return true;   // calibrating — hold the arbiter off, stay silent
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

        if (update_startup_chime(now_ms, can_fresh || usb_fresh || pwm_fresh || override)) {
            // Power-on chime owns the motor until it finishes or is pre-empted.
        } else if (override) {
            // VESC bench override owns the motor — leave it be.
        } else if (can_fresh) {
            motor_control.set_current(_can_amps);
        } else if (usb_fresh) {
            motor_control.set_current(usb_amps);
        } else if (pwm_fresh) {
            motor_control.set_current(_pwm_amps);
        } else {
            motor_control.set_current(0.0f);          // all sources stale → coast
        }
    }

    vesc_telem.update();
}

#endif  // HAL_PERIPH_ENABLE_FOC_ESC
