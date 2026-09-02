#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL.h>

namespace ChibiOS {

class MotorControl;

// Minimal VESC-protocol responder for VESC Tool's RT Data tab.
// Implements only COMM_FW_VERSION (0) and COMM_GET_VALUES (4).
// All other command IDs are silently dropped.
class VescTelemetry {
public:
    VescTelemetry(MotorControl &mc, uint8_t pole_pairs = 7) :
        _mc(mc), _pole_pairs(pole_pairs) {}

    void init(AP_HAL::UARTDriver *uart);
    void update();

    // ── l_max_duty unit conversion ──────────────────────────────────────────
    // VESC's l_max_duty caps `duty_now`, which is MODULATION DEPTH — roughly the
    // fraction of the available bus voltage, and what every VESC tuning guide
    // means by "max duty". This firmware's D_MAX is a PER-PHASE PWM on-time
    // ceiling (a hardware limit: MP1918 bootstrap refresh + the low-side window
    // the shunt ADC samples in). Because SVPWM sits centred at 0.5 duty and
    // swings symmetrically either side, the two differ by
    //     modulation = 2·(D_MAX − 0.5)          (MotorControl.cpp init)
    // so the numbers are NOT interchangeable: D_MAX 0.55 is 10% modulation, not
    // 55%. Passing the raw value both ways made VESC Tool's box mean something
    // ~9x smaller than it said, and a write from a stock VESC profile silently
    // crushed the voltage ceiling to ~1 V — the motor then spun but could not
    // accelerate, and flux-linkage detection could never reach its duty target.
    // Convert at the wire boundary so the box means what it claims.
    //
    // The hardware range D_MAX ∈ [0.55, 0.90] is modulation ∈ [0.10, 0.80].
    static constexpr float duty_max_to_vesc(float d_max) { return 2.0f * (d_max - 0.5f); }
    static constexpr float vesc_to_duty_max(float mod)   { return 0.5f + mod * 0.5f; }

    // ── Config bridge to the owning FOC_ESC (params) ─────────────────────────
    // Values that VESC Tool's "Write Motor Configuration" (COMM_SET_MCCONF) can
    // push. Only the fields that have a standard VESC mc_configuration slot are
    // here; our custom protections (stall/regen/vbus-fold/slew) have no VESC
    // field and stay DroneCAN/MAVLink-only params.
    struct McconfIn {
        float   motor_r;          // foc_motor_r [Ω]
        float   motor_l;          // foc_motor_l [H]
        float   motor_flux;       // foc_motor_flux_linkage [Wb]
        float   current_max;      // l_current_max [A] (motoring ceiling)
        // l_current_min [A], VESC stores it NEGATIVE. This is the braking/regen
        // ceiling, not a "negative iq" limit — mcpwm_foc.c:3651 flips the pair
        // with the direction of rotation so it caps braking either way round.
        // 0 = absent/invalid; the sink negates it to get a positive magnitude.
        float   current_min = 0.0f;
        float   abs_current_max;  // l_abs_current_max [A] → hard OC trip
        // l_max_vin [V]. In VESC this is the HARD over-voltage trip threshold
        // (mc_interface.c:1908 raises FAULT_CODE_OVER_VOLTAGE against it), NOT a
        // foldback ceiling — so it maps to V_OV. The graceful regen foldback has
        // its own pair of fields below, which is where V_MAX/V_FOLD live.
        float   max_vin;
        // l_battery_regen_cut_start/_end [V]: VESC's regen over-voltage cutoff
        // (mc_interface.c:2483), scaling braking to zero between the two. Our
        // V_MAX is the end and V_FOLD the width, so start = V_MAX - V_FOLD.
        // 0 = absent/invalid.
        float   regen_cut_start = 0.0f;
        float   regen_cut_end   = 0.0f;
        float   temp_fet_start;   // l_temp_fet_start [°C]
        float   temp_fet_end;     // l_temp_fet_end [°C]
        uint8_t poles;            // si_motor_poles (pole COUNT, = 2·pole_pairs)
        // l_max_duty as VESC means it: a MODULATION ceiling, already converted
        // out of the wire value by vesc_to_duty_max() — so this field carries a
        // D_MAX (per-phase) value ready to store. See the conversion helpers
        // above for why the two are not the same number. 0 = absent; the sink
        // range-checks against the hardware window before storing.
        float   max_duty = 0.0f;
        // l_duty_start → D_START. Fraction of the duty ceiling at which the
        // current limit starts folding back; VESC treats > 0.99 as disabled.
        // 0 = absent.
        float   duty_start = 0.0f;
        // l_max_erpm / l_min_erpm / l_erpm_start → MAX_ERPM / MIN_ERPM /
        // ERPM_START — the param names drop VESC's "l_" limits-group prefix
        // and are otherwise identical, so one name follows a value from the
        // VESC Tool box through the wire to Config and the ISR.
        // Motoring-current foldback against speed. Sign carries meaning: max is
        // the forward ceiling and min the reverse one, so 0 means absent for
        // both — a zero ceiling would be a drive that never turns, which is not
        // a configuration anyone is asking for over the wire.
        float   max_erpm   = 0.0f;
        float   min_erpm   = 0.0f;
        float   erpm_start = 0.0f;  // knee as a fraction of the limit (0 = absent)
        // Sensorless open-loop start, VESC foc_sl_openloop_* / foc_openloop_rpm.
        // All are meaningless at or below zero except rpm_low (0 is valid and is
        // the default), so 0 doubles as "absent" for the rest.
        float   ol_boost_q = 0.0f;  // foc_sl_openloop_boost_q [A]
        float   ol_max_q   = 0.0f;  // foc_sl_openloop_max_q   [A]
        float   ol_erpm    = 0.0f;  // foc_openloop_rpm    [eRPM]
        float   ol_rpm_low = -1.0f; // foc_openloop_rpm_low fraction (-1 = absent)
        float   ol_hyst    = -1.0f; // foc_sl_openloop_hyst      [s] (-1 = absent)
        float   ol_t_lock  = -1.0f; // foc_sl_openloop_time_lock [s] (-1 = absent)
        float   ol_t_ramp  = 0.0f;  // foc_sl_openloop_time_ramp [s]
        float   ol_t_const = 0.0f;  // foc_sl_openloop_time      [s]
        // foc_sensor_mode, VESC encoding: 0 = SENSORLESS, 1 = ENCODER, 2 = HALL,
        // 3 = HFI. 0xFF = "not supplied by this packet" so the sink can tell a
        // genuine 0 (sensorless) from an absent field and leave the param alone.
        // Only 0 and 2 are implemented here; the sink must ignore 1 and 3 rather
        // than falling through to sensorless on a mode we cannot actually run.
        uint8_t sensor_mode = 0xFF;
        float   observer_gain = 0.0f; // foc_observer_gain (0 = absent/invalid)
        float   current_kp    = 0.0f; // foc_current_kp  [V/A]     (0 = absent)
        float   current_ki    = 0.0f; // foc_current_ki  [V/(A·s)] (0 = absent)
        // Hall→observer blend band. VESC blends the commutation angle from pure
        // hall at foc_sl_erpm_start to pure observer at foc_sl_erpm
        // (foc_math.c foc_correct_hall: weight_hall = map(rpm, start, sl, 1, 0)).
        float   sl_erpm_start    = 0.0f;  // foc_sl_erpm_start  (0 = absent)
        float   sl_erpm          = 0.0f;  // foc_sl_erpm        (0 = absent)
        float   hall_interp_erpm = 0.0f;  // foc_hall_interp_erpm (0 = absent)
    };
    // What VESC Tool's App Settings → PPM page wrote (COMM_SET_APPCONF). Only the
    // fields the PWM throttle decode actually consumes are lifted out of the blob;
    // everything else is skipped. VESC sends the WHOLE appconf on every write, so
    // an untouched field must not overwrite something tuned here — hence the
    // absent sentinels, same convention as McconfIn.
    struct AppconfIn {
        // app_ppm_conf.ctrl_type, VESC's ppm_control_type enum. 0xFF = absent so
        // a genuine 0 (PPM_CTRL_TYPE_NONE) stays distinguishable from "not sent".
        uint8_t ppm_ctrl_type    = 0xFF;
        // app_ppm_conf.max_erpm_for_dir — speed above which the brake-to-reverse
        // gesture is refused. Only meaningful to CURRENT_BRAKE_REV_HYST.
        // 0 = absent (a zero ceiling would forbid reverse entirely, which is what
        // the NOREV modes are for).
        float   max_erpm_for_dir = 0.0f;
    };

    // Param-derived values the GET_MCCONF responder needs but MotorControl does
    // not expose. Pushed once at boot (reboot-to-apply model); poles also drives
    // the eRPM→RPM scaling and si_motor_poles field.
    struct ConfSnapshot {
        uint8_t poles          = 14;     // pole COUNT (7 pairs)
        float   max_vin        = 57.0f;  // → l_max_vin (hard OV trip, V_OV)
        float   regen_cut_start = 100.0f;// → l_battery_regen_cut_start (V_MAX - V_FOLD)
        float   regen_cut_end   = 110.0f;// → l_battery_regen_cut_end   (V_MAX)
        float   temp_fet_start = 85.0f;  // → l_temp_fet_start
        float   temp_fet_end   = 105.0f; // → l_temp_fet_end
        float   abs_current_max = 150.0f;// → l_abs_current_max
        float   observer_gain  = 2.5e7f; // → foc_observer_gain
        // PPM decode settings, echoed by the GET_APPCONF responder so VESC Tool's
        // App Settings → PPM page reads back what is actually running.
        uint8_t ppm_ctrl_type   = 3;      // → app_ppm_conf.ctrl_type (NOREV_BRAKE)
        float   max_erpm_for_dir = 4000.0f; // → app_ppm_conf.max_erpm_for_dir
    };
    void set_conf_snapshot(const ConfSnapshot &s) {
        _conf = s;
        if (s.poles >= 2) { _pole_pairs = s.poles / 2; }
    }
    // Register the sink called when VESC Tool writes a motor config. ctx is
    // passed back to cb (a static trampoline into the owning FOC_ESC).
    void set_mcconf_sink(void *ctx, void (*cb)(void *, const McconfIn &)) {
        _conf_ctx = ctx;
        _conf_cb  = cb;
    }
    // Sink for VESC Tool's "Write App Configuration". Shares _conf_ctx with the
    // mcconf sink — both trampoline into the same FOC_ESC.
    void set_appconf_sink(void *ctx, void (*cb)(void *, const AppconfIn &)) {
        _conf_ctx = ctx;
        _appconf_cb = cb;
    }
    // Refresh just the PPM half of the snapshot, without a reboot. The rest of
    // ConfSnapshot is boot-time by nature (reboot-to-apply), but these two are
    // read back by VESC Tool the moment a write completes — the tool re-reads to
    // confirm, and that read lands BEFORE the deferred reboot. Serving the boot
    // snapshot there reports the old value milliseconds after a successful
    // write, which is indistinguishable from the write having been rejected.
    // Param storage exhausted (AP_Param::get_eeprom_full). Pushed in from the
    // owner rather than read here, to keep AP_Param out of the HAL layer. When
    // this is set, every set_and_save() is a silent no-op — which is the one
    // failure that makes a correct config path look broken.
    void set_storage_full(bool full) { _storage_full = full; }
    // The sink accepted a ctrl_type and called set_and_save(). Separates "the
    // wire delivered a value we refused" from "we saved it and it did not stick".
    void note_appconf_accepted() { _appconf_ok++; }
    void set_ppm_conf(uint8_t ctrl_type, float max_erpm_for_dir) {
        _conf.ppm_ctrl_type    = ctrl_type;
        _conf.max_erpm_for_dir = max_erpm_for_dir;
    }

    // ── Parameter-detection bridge (VESC Tool FOC tab / motor wizard) ────────
    // What VESC Tool asked for. VESC treats these as "blocking commands" run in
    // their own thread, replying only when the measurement finishes
    // (comm/commands.c) — the tool waits, so we may answer many seconds later
    // from a state machine instead of stalling the periph loop.
    enum class DetectKind : uint8_t { NONE, R_L, FLUX_OPENLOOP, APPLY_ALL_FOC };
    struct DetectReq {
        DetectKind kind = DetectKind::NONE;
        // FLUX_OPENLOOP / APPLY_ALL_FOC
        float current      = 0.0f;   // injection current [A]
        float erpm_per_sec = 0.0f;   // speed ramp rate [eRPM/s]
        float duty         = 0.0f;   // ramp stops when duty reaches this (VESC convention)
        float resistance   = 0.0f;   // [Ω]  (0 = use configured)
        float inductance   = 0.0f;   // [H]  (0 = use configured)
        // APPLY_ALL_FOC only
        float max_power_loss = 0.0f; // [W] — sizes the detection current
        float openloop_erpm  = 0.0f;
        float sl_erpm        = 0.0f;
    };
    void set_detect_sink(void *ctx, void (*cb)(void *, const DetectReq &)) {
        _detect_ctx = ctx;
        _detect_cb  = cb;
    }
    // Replies, emitted by the detection state machine when it finishes. Formats
    // are fixed by comm/commands.c and must match exactly or VESC Tool ignores
    // them. A failed measurement reports zeros, which is how VESC signals it too.
    void send_detect_r_l(float r, float l, float ld_lq_diff);
    void send_detect_flux(float linkage);
    void send_detect_apply_all(int16_t result);
    // Emit one COMM_PRINT line (VESC Tool's terminal). Public so the detection
    // state machine, which lives in FOC_ESC, can explain its own failures —
    // the reply formats above carry a result but no reason.
    void send_print(const char *s);

    // ── Throttle-arbiter interface (see AP_Periph_FW::update_motor_test) ─────
    // USB torque source: holds the last COMM_SET_CURRENT value for as long as the
    // host LINK is alive (any valid packet within timeout_ms), matching VESC's
    // setpoint-hold model — VESC Tool sends SET_CURRENT sparsely and keeps the
    // link alive with GET_VALUES/COMM_ALIVE polling, relying on the firmware to
    // hold the setpoint between sends (expiring on the SET_CURRENT age instead
    // aborts a start mid-forced-spin: "kicks but never spins"). Requires at least
    // one real SET_CURRENT (_usb_current_ms != 0) so a purely passive poller (RT
    // monitoring, no command) never owns the motor. Lower priority than CAN.
    bool usb_current(uint32_t now_ms, uint16_t timeout_ms, float &amps) const {
        if (_usb_current_ms != 0 && (now_ms - _host_alive_ms) < timeout_ms) {
            amps = _usb_current_a;
            return true;
        }
        return false;
    }
    // True while a VESC Tool bench override (rpm / brake / duty-debug) is active.
    // These modes drive MotorControl directly, so the arbiter stands off rather
    // than stomping them with a current command.
    //
    // Keyed on LINK freshness (_host_alive_ms), not on the age of the override
    // setpoint — exactly as usb_current() above, and for the same reason. VESC
    // Tool sends SET_RPM / SET_DUTY / SET_CURRENT_BRAKE ONCE and then merely
    // polls; VESC's own model is that the setpoint persists until changed or the
    // link dies. Expiring on setpoint age instead meant the override lapsed
    // ~200 ms after the single RPM packet, the arbiter fell through to the still
    // -live USB current source, and set_current() switched the controller out of
    // SPEED mode — the motor started and immediately stopped.
    //
    // _override_ms is cleared by an explicit zero/release command so ordinary
    // CAN/PWM arbitration resumes rather than being locked out for the whole
    // time VESC Tool stays connected.
    bool override_active(uint32_t now_ms, uint16_t timeout_ms) const {
        return _override_ms != 0 && (now_ms - _host_alive_ms) < timeout_ms;
    }

private:
    enum class RxState : uint8_t {
        WAIT_START,
        WAIT_LEN_SHORT,
        WAIT_LEN_LONG_HI,
        WAIT_LEN_LONG_LO,
        WAIT_PAYLOAD,
        WAIT_CRC_HI,
        WAIT_CRC_LO,
        WAIT_END,
    };

    void feed_byte(uint8_t b);
    void dispatch();
    void send_packet(const uint8_t *payload, uint16_t len);

    void handle_fw_version();
    void handle_get_values();
    void handle_set_rpm();
    void handle_set_current();
    void handle_set_current_brake();
    void handle_set_duty();
    void handle_detect_hall();
    // COMM_GET_MCCONF / _DEFAULT: serialize the full mc_configuration blob (VESC
    // FW 6.06 layout) so VESC Tool's "Read Motor Configuration" succeeds and its
    // FOC → Hall Sensors tab shows the stored table. reply_id echoes the request.
    void handle_get_mcconf(uint8_t reply_id);
    // COMM_SET_MCCONF: parse VESC Tool's "Write Motor Configuration" blob (6.06
    // layout), extract the standard fields we back with params, and hand them to
    // the registered sink (which persists them and reboots to apply).
    void handle_set_mcconf();
    // COMM_GET_APPCONF / _DEFAULT: emit a complete, layout-valid app_configuration
    // so VESC Tool's App Settings pages open. Almost every field is a fixed
    // stand-in — only the PPM block reflects anything real here, because the PPM
    // page is the one that configures something this firmware implements.
    void handle_get_appconf(uint8_t reply_id);
    // COMM_SET_APPCONF: lift app_ppm_conf.ctrl_type and max_erpm_for_dir out of
    // VESC Tool's "Write App Configuration" blob and hand them to the sink.
    void handle_set_appconf();
    // VESC-Tool terminal (COMM_TERMINAL_CMD): a tiny command set to read/trigger
    // the hall table (works even if a VESC Tool version can't read MCCONF).
    void handle_terminal();
    void print_hall_table();
    // 'diag' terminal command: dump the throttle-arbiter decision inputs.
    void print_diag();
    // Dump MotorControl's latched trip snapshot — what the ISR saw at the tick a
    // fault fired. The one readout that can tell a real overcurrent from a sense
    // artifact, and say whether the commutation angle had come apart first.
    void print_trip();

// ─── FOC_WATCH ─── temporary diagnostic, remove as a unit (see MotorControl.h)
// Not #if-guarded: this header only forward-declares MotorControl, so the
// FOC_WATCH define isn't visible here and adding the include just to see it
// would be a coupling change to undo later. The switch guards the CODE (in the
// .cpp and in MotorControl.h); with it off these three declarations are inert.
    // `watch [seconds]` — stream the angle sources and dq currents as COMM_PRINT
    // lines at WATCH_HZ so they can be read while the motor turns. 0 stops it.
    void     print_watch();
    uint32_t _watch_until_ms = 0;   // streaming while millis() < this
    uint32_t _watch_next_ms  = 0;   // next line due
// ─── end FOC_WATCH ───────────────────────────────────────────────────────────

    static uint16_t crc16(const uint8_t *data, uint16_t len);

    AP_HAL::UARTDriver *_uart = nullptr;
    MotorControl       &_mc;
    uint8_t             _pole_pairs;

    RxState  _state = RxState::WAIT_START;
    uint16_t _payload_len = 0;
    uint16_t _payload_idx = 0;
    uint16_t _rx_crc = 0;
    // Sized for the largest inbound frame — the ~470-byte COMM_SET_MCCONF blob
    // (VESC Tool "Write Motor Configuration"), a long (0x03-framed) packet.
    uint8_t  _payload[512];

    // Sized for the largest reply — the ~490-byte COMM_GET_MCCONF blob plus
    // long-frame header (3) + CRC (2) + stop (1).
    uint8_t  _tx_buf[512];

    // Config bridge (see set_conf_snapshot / set_mcconf_sink).
    ConfSnapshot _conf;
    void        *_conf_ctx = nullptr;
    void       (*_conf_cb)(void *, const McconfIn &) = nullptr;
    void       (*_appconf_cb)(void *, const AppconfIn &) = nullptr;
    // ── COMM_SET_APPCONF forensics, printed by the 'diag' terminal command ──
    // An app-config write that does not take is otherwise completely invisible:
    // the tool reports success off the ack and the value just reads back
    // unchanged, which looks identical whether the frame never arrived, failed
    // the signature, or was parsed and then refused. These separate those cases.
    uint16_t _appconf_rx     = 0;     // SET_APPCONF frames that reached the handler
    uint16_t _appconf_badsig = 0;     // ...of those, rejected on the signature
    uint16_t _appconf_short  = 0;     // ...rejected as truncated
    uint8_t  _appconf_ctrl   = 0xFF;  // ctrl_type parsed from the last good frame
    float    _appconf_dir    = 0.0f;  // max_erpm_for_dir likewise
    bool     _storage_full   = false; // AP_Param storage exhausted (see set_storage_full)
    uint16_t _appconf_ok     = 0;     // ctrl_type values passed to set_and_save()
    // Parameter-detection bridge (see set_detect_sink).
    void        *_detect_ctx = nullptr;
    void       (*_detect_cb)(void *, const DetectReq &) = nullptr;
    void         handle_detect(uint8_t id);

    // Throttle-arbiter state (thread context).
    float    _usb_current_a  = 0.0f;  // last COMM_SET_CURRENT value [A]
    uint32_t _usb_current_ms = 0;     // millis() of that command (0 = none ever sent)
    uint32_t _host_alive_ms  = 0;     // millis() of the last valid packet (link keepalive)
    uint32_t _override_ms    = 0;     // millis() of last rpm/brake/duty override (0 = none)
    bool     _hall_detect_pending = false; // a hall-detect spin is running; emit table when done
    uint32_t _hall_detect_ms      = 0;     // millis() the spin was started (for the give-up timeout)
};

} // namespace ChibiOS
