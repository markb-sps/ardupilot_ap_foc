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
};

} // namespace ChibiOS
