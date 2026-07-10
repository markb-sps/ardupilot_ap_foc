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
    // USB torque source: true (and amps set) if a COMM_SET_CURRENT arrived within
    // timeout_ms. A current command drives the motor through the arbiter (subject
    // to CAN priority), not directly, so CAN can cleanly override it.
    bool usb_current(uint32_t now_ms, uint16_t timeout_ms, float &amps) const {
        if (_usb_current_ms != 0 && (now_ms - _usb_current_ms) < timeout_ms) {
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

    static uint16_t crc16(const uint8_t *data, uint16_t len);

    AP_HAL::UARTDriver *_uart = nullptr;
    MotorControl       &_mc;
    uint8_t             _pole_pairs;

    RxState  _state = RxState::WAIT_START;
    uint16_t _payload_len = 0;
    uint16_t _payload_idx = 0;
    uint16_t _rx_crc = 0;
    uint8_t  _payload[80];

    uint8_t  _tx_buf[128];

    // Throttle-arbiter state (thread context).
    float    _usb_current_a  = 0.0f;  // last COMM_SET_CURRENT value [A]
    uint32_t _usb_current_ms = 0;     // millis() of that command (0 = none yet)
    uint32_t _override_ms    = 0;     // millis() of last rpm/brake/duty override (0 = none)
};

} // namespace ChibiOS
