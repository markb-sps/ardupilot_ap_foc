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
    bool override_active(uint32_t now_ms, uint16_t timeout_ms) const {
        return _override_ms != 0 && (now_ms - _override_ms) < timeout_ms;
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
    // FW 6.00 layout) so VESC Tool's "Read Motor Configuration" succeeds and its
    // FOC → Hall Sensors tab shows the stored table. reply_id echoes the request.
    void handle_get_mcconf(uint8_t reply_id);
    // VESC-Tool terminal (COMM_TERMINAL_CMD): a tiny command set to read/trigger
    // the hall table (works even if a VESC Tool version can't read MCCONF).
    void handle_terminal();
    void print_hall_table();
    void send_print(const char *s);   // emit one COMM_PRINT line

    static uint16_t crc16(const uint8_t *data, uint16_t len);

    AP_HAL::UARTDriver *_uart = nullptr;
    MotorControl       &_mc;
    uint8_t             _pole_pairs;

    RxState  _state = RxState::WAIT_START;
    uint16_t _payload_len = 0;
    uint16_t _payload_idx = 0;
    uint16_t _rx_crc = 0;
    uint8_t  _payload[80];

    // Sized for the largest reply — the ~482-byte COMM_GET_MCCONF blob plus
    // long-frame header (3) + CRC (2) + stop (1).
    uint8_t  _tx_buf[512];

    // Throttle-arbiter state (thread context).
    float    _usb_current_a  = 0.0f;  // last COMM_SET_CURRENT value [A]
    uint32_t _usb_current_ms = 0;     // millis() of that command (0 = none ever sent)
    uint32_t _host_alive_ms  = 0;     // millis() of the last valid packet (link keepalive)
    uint32_t _override_ms    = 0;     // millis() of last rpm/brake/duty override (0 = none)
    bool     _hall_detect_pending = false; // a hall-detect spin is running; emit table when done
};

} // namespace ChibiOS
