#include "VescTelemetry.h"
#include "MotorControl.h"

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <hal.h>
#include <string.h>
#include <math.h>

namespace ChibiOS {

namespace {

constexpr uint8_t COMM_FW_VERSION = 0;
constexpr uint8_t COMM_GET_VALUES = 4;
constexpr uint8_t COMM_SET_DUTY   = 5;
constexpr uint8_t COMM_SET_CURRENT = 6;
constexpr uint8_t COMM_SET_RPM    = 8;

constexpr uint8_t FW_MAJOR = 6;
constexpr uint8_t FW_MINOR = 0;
constexpr const char HW_NAME[] = "AP_FOC";

// STM32G4 unique device ID: 96 bits at 0x1FFF7590
constexpr uintptr_t STM32G4_UID_BASE = 0x1FFF7590UL;

// Pack helpers — VESC uses big-endian on the wire.
inline void put_u8 (uint8_t *&p, uint8_t  v) { *p++ = v; }
inline void put_i16(uint8_t *&p, int16_t  v) {
    *p++ = uint8_t(v >> 8); *p++ = uint8_t(v);
}
inline void put_i32(uint8_t *&p, int32_t  v) {
    *p++ = uint8_t(v >> 24); *p++ = uint8_t(v >> 16);
    *p++ = uint8_t(v >> 8);  *p++ = uint8_t(v);
}
inline void put_f16(uint8_t *&p, float v, float scale) {
    int16_t i = int16_t(lroundf(v * scale));
    put_i16(p, i);
}
inline void put_f32(uint8_t *&p, float v, float scale) {
    int32_t i = int32_t(lroundf(v * scale));
    put_i32(p, i);
}

} // namespace

// CCITT-16, poly 0x1021, init 0x0000 (matches vedderb/bldc comm/crc.c table)
uint16_t VescTelemetry::crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= uint16_t(data[i]) << 8;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x8000U) ? uint16_t((crc << 1) ^ 0x1021U) : uint16_t(crc << 1);
        }
    }
    return crc;
}

void VescTelemetry::init(AP_HAL::UARTDriver *uart)
{
    _uart = uart;
    if (_uart != nullptr) {
        // VESC Tool over USB CDC ignores baud, but bigger buffers matter:
        // GET_VALUES response is ~80 bytes and arrives at 50–100 Hz.
        _uart->begin(115200, 256, 512);
    }
}

void VescTelemetry::update()
{
    if (_uart == nullptr) {
        return;
    }
    uint32_t avail = _uart->available();
    // Cap per-call work to avoid starving the periph loop.
    if (avail > 256) avail = 256;
    while (avail-- > 0) {
        uint8_t b;
        if (!_uart->read(b)) break;
        feed_byte(b);
    }
}

void VescTelemetry::feed_byte(uint8_t b)
{
    switch (_state) {
    case RxState::WAIT_START:
        // Only the short-packet start (0x02) is accepted. The host never sends
        // long packets, and 0x03 is ALSO every frame's end byte — accepting it
        // as a "long length" start makes a single misaligned byte latch onto a
        // trailing 0x03, mis-read the next two bytes as a huge length, and
        // swallow whole frames until it luckily realigns. Ignoring everything
        // but 0x02 here means any junk byte just keeps us waiting for the next
        // real start, so the stream resyncs within one packet.
        if (b == 0x02) {
            _state = RxState::WAIT_LEN_SHORT;
        }
        break;

    case RxState::WAIT_LEN_SHORT:
        _payload_len = b;
        _payload_idx = 0;
        if (_payload_len == 0 || _payload_len > sizeof(_payload)) {
            _state = RxState::WAIT_START;
        } else {
            _state = RxState::WAIT_PAYLOAD;
        }
        break;

    case RxState::WAIT_LEN_LONG_HI:
        _payload_len = uint16_t(b) << 8;
        _state = RxState::WAIT_LEN_LONG_LO;
        break;

    case RxState::WAIT_LEN_LONG_LO:
        _payload_len |= b;
        _payload_idx = 0;
        if (_payload_len == 0 || _payload_len > sizeof(_payload)) {
            _state = RxState::WAIT_START;
        } else {
            _state = RxState::WAIT_PAYLOAD;
        }
        break;

    case RxState::WAIT_PAYLOAD:
        _payload[_payload_idx++] = b;
        if (_payload_idx >= _payload_len) {
            _state = RxState::WAIT_CRC_HI;
        }
        break;

    case RxState::WAIT_CRC_HI:
        _rx_crc = uint16_t(b) << 8;
        _state = RxState::WAIT_CRC_LO;
        break;

    case RxState::WAIT_CRC_LO:
        _rx_crc |= b;
        _state = RxState::WAIT_END;
        break;

    case RxState::WAIT_END:
        if (b == 0x03 && _rx_crc == crc16(_payload, _payload_len)) {
            dispatch();
        }
        _state = RxState::WAIT_START;
        break;
    }
}

void VescTelemetry::dispatch()
{
    if (_payload_len < 1) return;
    // Any CRC-valid packet (set-point, GET_VALUES poll, COMM_ALIVE) means the
    // host is alive — pet the comms failsafe so it only coasts on real loss.
    _mc.notify_host_alive();
    switch (_payload[0]) {
    case COMM_FW_VERSION:
        handle_fw_version();
        break;
    case COMM_GET_VALUES:
        handle_get_values();
        break;
    case COMM_SET_DUTY:
        handle_set_duty();
        break;
    case COMM_SET_CURRENT:
        handle_set_current();
        break;
    case COMM_SET_RPM:
        handle_set_rpm();
        break;
    default:
        break; // silently drop everything else
    }
}

void VescTelemetry::send_packet(const uint8_t *payload, uint16_t len)
{
    if (_uart == nullptr || len == 0) return;

    const uint16_t crc = crc16(payload, len);
    uint8_t *p = _tx_buf;

    if (len <= 255) {
        *p++ = 0x02;
        *p++ = uint8_t(len);
    } else {
        *p++ = 0x03;
        *p++ = uint8_t(len >> 8);
        *p++ = uint8_t(len);
    }
    if ((p - _tx_buf) + len + 3 > int(sizeof(_tx_buf))) {
        return; // would overflow; drop
    }
    memcpy(p, payload, len);
    p += len;
    *p++ = uint8_t(crc >> 8);
    *p++ = uint8_t(crc);
    *p++ = 0x03;

    _uart->write(_tx_buf, size_t(p - _tx_buf));
}

void VescTelemetry::handle_fw_version()
{
    uint8_t buf[64];
    uint8_t *p = buf;

    put_u8(p, COMM_FW_VERSION);
    put_u8(p, FW_MAJOR);
    put_u8(p, FW_MINOR);

    const size_t hwname_len = sizeof(HW_NAME); // includes terminator
    memcpy(p, HW_NAME, hwname_len);
    p += hwname_len;

    // 12-byte UUID from STM32 unique ID region
    const uint8_t *uid = reinterpret_cast<const uint8_t *>(STM32G4_UID_BASE);
    memcpy(p, uid, 12);
    p += 12;

    put_u8(p, 0); // pairing_done
    put_u8(p, 0); // fw_test_version
    put_u8(p, 0); // hw_type = HW_TYPE_VESC
    put_u8(p, 0); // custom_config_num

    send_packet(buf, uint16_t(p - buf));
}

void VescTelemetry::handle_get_values()
{
    uint8_t buf[96];
    uint8_t *p = buf;

    float id = 0, iq = 0;
    _mc.get_idq(id, iq);
    float vd = 0, vq = 0;
    _mc.get_vdq(vd, vq);
    const float theta     = _mc.get_estimated_angle();      // control angle [rad]
    const float obs_theta = _mc.get_observer_angle();       // observer angle [rad]
    const float free_theta = _mc.get_free_observer_angle(); // unseeded shadow observer [rad]

    const float i_motor = _mc.get_motor_current(); // q-axis (torque) current [A]
    const float v_in    = _mc.read_vbus();   // fresh ADC sample (PA0 divider)
    const float duty    = _mc.get_duty();
    const float erpm    = _mc.get_erpm();          // electrical RPM (VESC convention)

    put_u8 (p, COMM_GET_VALUES);
    put_f16(p, 25.0f,    10.0f);   // temp_mos   (no sensor yet — placeholder)
    put_f16(p, 25.0f,    10.0f);   // temp_motor (no sensor yet — placeholder)
    put_f32(p, i_motor,  100.0f);  // current_motor [A * 100]
    put_f32(p, 0.0f,     100.0f);  // current_in (not measured)
    put_f32(p, id,       100.0f);  // id
    put_f32(p, iq,       100.0f);  // iq
    put_f16(p, duty,     1000.0f); // duty
    put_f32(p, erpm,     1.0f);    // rpm (electrical)
    put_f16(p, v_in,     10.0f);   // v_in
    put_f32(p, 0.0f,     10000.0f);// amp_hours
    put_f32(p, 0.0f,     10000.0f);// amp_hours_charged
    put_f32(p, 0.0f,     10000.0f);// watt_hours
    put_f32(p, 0.0f,     10000.0f);// watt_hours_charged
    put_i32(p, _mc.get_state());   // tachometer ← FOC state (0=IDLE 1=ALIGN 2=OPENLOOP 3=BLEND 4=CLOSED 5=FAULT 6=DEBUG)
    put_i32(p, 0);                 // tachometer_abs
    put_u8 (p, _mc.get_fault());   // fault_code
    put_f32(p, 0.0f,     1000000.0f); // pid_pos (position control not used)
    put_u8 (p, 0);                 // controller_id (vesc_id)
    put_f16(p, 25.0f,    10.0f);   // temp_mos_1
    put_f16(p, 25.0f,    10.0f);   // temp_mos_2
    put_f16(p, 25.0f,    10.0f);   // temp_mos_3
    put_f32(p, vd,       1000.0f); // vd
    put_f32(p, vq,       1000.0f); // vq
    put_u8 (p, _mc.get_state());   // status (reuse for FOC state machine)
    // Bring-up extras appended past the standard packet (VESC Tool ignores the
    // trailing bytes; our bench scripts read them). Both angles in rad·10000.
    put_f32(p, theta,      10000.0f); // control/commanded angle      (offset 74)
    put_f32(p, obs_theta,  10000.0f); // observer-estimated angle     (offset 78)
    put_f32(p, free_theta, 10000.0f); // unseeded shadow observer     (offset 82)

    send_packet(buf, uint16_t(p - buf));
}

// COMM_SET_CURRENT: payload = [cmd, int32 current_mA] — direct iq (torque)
// control. Best for bench-debugging the current loop in isolation.
void VescTelemetry::handle_set_current()
{
    if (_payload_len < 5) return;
    const int32_t ma = (int32_t(_payload[1]) << 24) |
                       (int32_t(_payload[2]) << 16) |
                       (int32_t(_payload[3]) <<  8) |
                        int32_t(_payload[4]);
    _mc.set_current(float(ma) / 1000.0f);
}

// COMM_SET_DUTY: payload = [cmd, int32 duty·1e5]. Repurposed as the open-loop
// voltage BRING-UP control: |duty|→modulation, sign→direction, fixed slow spin.
// Use VESC Tool's Duty slider at a few % to verify current sign/scale and that
// the observer angle tracks, before trusting the closed loop.
void VescTelemetry::handle_set_duty()
{
    if (_payload_len < 5) return;
    const int32_t d = (int32_t(_payload[1]) << 24) |
                      (int32_t(_payload[2]) << 16) |
                      (int32_t(_payload[3]) <<  8) |
                       int32_t(_payload[4]);
    _mc.set_debug_voltage(float(d) / 100000.0f);
}

// COMM_SET_RPM: payload = [cmd, int32 erpm] (big-endian, electrical RPM).
void VescTelemetry::handle_set_rpm()
{
    if (_payload_len < 5) return;
    const int32_t erpm = (int32_t(_payload[1]) << 24) |
                         (int32_t(_payload[2]) << 16) |
                         (int32_t(_payload[3]) <<  8) |
                          int32_t(_payload[4]);
    _mc.set_rpm(float(erpm));
}

} // namespace ChibiOS

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
