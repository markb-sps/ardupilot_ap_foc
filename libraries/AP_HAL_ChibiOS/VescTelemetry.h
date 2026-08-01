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

    // ── Config bridge to the owning FOC_ESC (params) ─────────────────────────
    // Values that VESC Tool's "Write Motor Configuration" (COMM_SET_MCCONF) can
    // push. Only the fields that have a standard VESC mc_configuration slot are
    // here; our custom protections (stall/regen/vbus-fold/slew) have no VESC
    // field and stay DroneCAN/MAVLink-only params.
    struct McconfIn {
        float   motor_r;          // foc_motor_r [Ω]
        float   motor_l;          // foc_motor_l [H]
        float   motor_flux;       // foc_motor_flux_linkage [Wb]
        float   current_max;      // l_current_max [A]
        float   abs_current_max;  // l_abs_current_max [A] → hard OC trip
        float   max_vin;          // l_max_vin [V] → vbus_max
        float   temp_fet_start;   // l_temp_fet_start [°C]
        float   temp_fet_end;     // l_temp_fet_end [°C]
        uint8_t poles;            // si_motor_poles (pole COUNT, = 2·pole_pairs)
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
    // Param-derived values the GET_MCCONF responder needs but MotorControl does
    // not expose. Pushed once at boot (reboot-to-apply model); poles also drives
    // the eRPM→RPM scaling and si_motor_poles field.
    struct ConfSnapshot {
        uint8_t poles          = 14;     // pole COUNT (7 pairs)
        float   max_vin        = 57.0f;  // → l_max_vin
        float   temp_fet_start = 85.0f;  // → l_temp_fet_start
        float   temp_fet_end   = 105.0f; // → l_temp_fet_end
        float   abs_current_max = 150.0f;// → l_abs_current_max
        float   observer_gain  = 2.5e7f; // → foc_observer_gain
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
    // VESC-Tool terminal (COMM_TERMINAL_CMD): a tiny command set to read/trigger
    // the hall table (works even if a VESC Tool version can't read MCCONF).
    void handle_terminal();
    void print_hall_table();
    // 'diag' terminal command: dump the throttle-arbiter decision inputs.
    void print_diag();
    void send_print(const char *s);   // emit one COMM_PRINT line

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
};

} // namespace ChibiOS
