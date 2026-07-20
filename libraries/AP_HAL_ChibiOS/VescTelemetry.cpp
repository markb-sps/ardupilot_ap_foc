#include "VescTelemetry.h"
#include "MotorControl.h"

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <hal.h>
#include <string.h>
#include <math.h>

extern const AP_HAL::HAL& hal;

namespace ChibiOS {

namespace {

constexpr uint8_t COMM_FW_VERSION = 0;
constexpr uint8_t COMM_GET_VALUES = 4;
constexpr uint8_t COMM_SET_DUTY   = 5;
constexpr uint8_t COMM_SET_CURRENT = 6;
constexpr uint8_t COMM_SET_CURRENT_BRAKE = 7;
constexpr uint8_t COMM_SET_RPM    = 8;
constexpr uint8_t COMM_TERMINAL_CMD = 20;   // VESC-Tool terminal command (ascii)
constexpr uint8_t COMM_PRINT        = 21;   // terminal text response
// Real VESC COMM_PACKET_ID value (datatypes.h: FW_VERSION=0 … SAMPLE_PRINT=19,
// TERMINAL_CMD=20, PRINT=21, ROTOR_POSITION=22, EXPERIMENT_SAMPLE=23,
// DETECT_MOTOR_PARAM=24, DETECT_MOTOR_R_L=25, DETECT_MOTOR_FLUX_LINKAGE=26,
// DETECT_ENCODER=27, DETECT_HALL_FOC=28). This is what VESC Tool's hall detect
// button sends; must match exactly or the command is silently dropped.
constexpr uint8_t COMM_DETECT_HALL_FOC = 28;
constexpr uint8_t COMM_GET_MCCONF         = 14;  // VESC Tool "Read Motor Configuration"
constexpr uint8_t COMM_GET_MCCONF_DEFAULT = 15;

constexpr uint8_t FW_MAJOR = 6;
constexpr uint8_t FW_MINOR = 0;
// mc_configuration serialization signature for VESC FW 6.00 (confgenerator.h
// MCCONF_SIGNATURE). VESC Tool validates this before parsing the blob, so it is
// version-locked: it must match the FW version reported by COMM_FW_VERSION (6.0)
// AND the config layout emitted by handle_get_mcconf() below, field-for-field.
constexpr uint32_t MCCONF_SIGNATURE = 776184161u;
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
inline void put_u16(uint8_t *&p, uint16_t v) {
    *p++ = uint8_t(v >> 8); *p++ = uint8_t(v);
}
inline void put_u32(uint8_t *&p, uint32_t v) {
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
// VESC buffer_append_float32_auto: sign + 8-bit exponent + 23-bit mantissa
// packed into a uint32 (its own compressed float, not IEEE-754). Must match
// bit-for-bit or VESC Tool mis-decodes every f32_auto field after it.
inline void put_f32_auto(uint8_t *&p, float number) {
    int e = 0;
    float sig = frexpf(number, &e);
    float sig_abs = fabsf(sig);
    uint32_t sig_i = 0;
    if (sig_abs >= 0.5f) {
        sig_i = uint32_t((sig_abs - 0.5f) * 2.0f * 8388608.0f);
        e += 126;
    }
    uint32_t res = (uint32_t(e & 0xFF) << 23) | (sig_i & 0x7FFFFF);
    if (sig < 0.0f) res |= (1u << 31);
    put_u32(p, res);
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
    // Hall-table detection runs asynchronously (~2 s current-controlled spin in the ISR).
    // While it is in progress keep the bench-override flag fresh so the throttle
    // arbiter stands off (its own timeout is only ~200 ms), and emit the result
    // once in VESC Tool's native COMM_DETECT_HALL_FOC reply format:
    //   [id, hall_tab[0..7] uint8 (angle·200/360; 255 = invalid), res (0 = ok)].
    if (_hall_detect_pending) {
        _override_ms = AP_HAL::millis();
        float deg[8];
        if (_mc.hall_detect_result(deg)) {
            uint8_t buf[1 + 8 + 1];
            uint8_t *p = buf;
            put_u8(p, COMM_DETECT_HALL_FOC);
            uint8_t valid = 0;
            for (uint8_t k = 0; k < 8; k++) {
                if (isnan(deg[k])) {
                    put_u8(p, 255);
                } else {
                    float a = deg[k];
                    while (a < 0.0f)    a += 360.0f;
                    while (a >= 360.0f) a -= 360.0f;
                    put_u8(p, uint8_t(lroundf(a * (200.0f / 360.0f)) % 200));
                    valid++;
                }
            }
            put_u8(p, valid >= 6 ? 0 : 1);   // res: 0 = all six states seen
            send_packet(buf, uint16_t(p - buf));
            _hall_detect_pending = false;
        }
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
    // host is alive — pet the comms failsafe so it only coasts on real loss, and
    // keep the held USB current setpoint alive (VESC setpoint-hold; see usb_current).
    _mc.notify_host_alive();
    _host_alive_ms = AP_HAL::millis();
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
    case COMM_SET_CURRENT_BRAKE:
        handle_set_current_brake();
        break;
    case COMM_SET_RPM:
        handle_set_rpm();
        break;
    case COMM_DETECT_HALL_FOC:
        handle_detect_hall();
        break;
    case COMM_GET_MCCONF:
    case COMM_GET_MCCONF_DEFAULT:
        // VESC Tool reads the motor config (incl. the FOC hall table) via this.
        // Reply id echoes the request so the tool routes it correctly.
        handle_get_mcconf(_payload[0]);
        break;
    case COMM_TERMINAL_CMD:
        handle_terminal();
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
    uint8_t buf[104];
    uint8_t *p = buf;

    float id = 0, iq = 0;
    _mc.get_idq(id, iq);
    float ia = 0, ib = 0, ic = 0;
    _mc.get_phase_currents(ia, ib, ic);
    float vd = 0, vq = 0;
    _mc.get_vdq(vd, vq);
    const float theta     = _mc.get_estimated_angle();      // control angle [rad]
    const float obs_theta = _mc.get_observer_angle();       // observer angle [rad]

    const float i_motor = _mc.get_motor_current(); // q-axis (torque) current [A]
    const float v_in    = _mc.read_vbus();   // fresh ADC sample (PA0 divider)
    const float duty    = _mc.get_duty();
    const float erpm    = _mc.get_erpm();          // electrical RPM (VESC convention)

    const float temp_fet = _mc.get_fet_temp();     // board NTC by the bridge [°C]

    put_u8 (p, COMM_GET_VALUES);
    put_f16(p, temp_fet, 10.0f);   // temp_mos
    put_f16(p, 25.0f,    10.0f);   // temp_motor (no sensor — placeholder)
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
    put_f32(p, 0.0f,       10000.0f); // reserved (was shadow observer) (offset 82)
    put_f32(p, ia,           100.0f); // raw phase-U current [A]      (offset 86)
    put_f32(p, ib,           100.0f); // raw phase-V current [A]      (offset 90)
    put_f32(p, ic,           100.0f); // measured phase-W current [A] (offset 94)
    put_f32(p, _mc.get_phase_residual(), 100.0f); // ia+ib+ic [A]     (offset 98)
    put_u8 (p, _mc.get_hall_state());  // live raw hall state 0..7     (offset 102)
    put_u8 (p, _mc.hall_table_valid() ? 1 : 0); // hall table loaded?  (offset 103)

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
    // Route through the throttle arbiter (as the USB source) rather than driving
    // MotorControl directly, so CAN can take priority. Applied in update_motor_test().
    _usb_current_a  = float(ma) / 1000.0f;
    _usb_current_ms = AP_HAL::millis();
}

// COMM_SET_CURRENT_BRAKE: payload = [cmd, int32 brake_mA] — regen brake
// magnitude; sign auto-applied opposite to rotation in the ISR.
void VescTelemetry::handle_set_current_brake()
{
    if (_payload_len < 5) return;
    const int32_t ma = (int32_t(_payload[1]) << 24) |
                       (int32_t(_payload[2]) << 16) |
                       (int32_t(_payload[3]) <<  8) |
                        int32_t(_payload[4]);
    _mc.set_brake_current(float(ma) / 1000.0f);
    _override_ms = AP_HAL::millis();   // bench override: arbiter stands off
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
    _override_ms = AP_HAL::millis();   // bench override: arbiter stands off
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
    _override_ms = AP_HAL::millis();   // bench override: arbiter stands off
}

// COMM_DETECT_HALL_FOC: start a hall-table detection spin (payload current arg
// ignored — the firmware uses its own fixed low-modulation open-loop spin). The
// result is emitted asynchronously from update() when the spin completes.
void VescTelemetry::handle_detect_hall()
{
    _mc.start_hall_detect();
    _hall_detect_pending = true;
    _override_ms = AP_HAL::millis();   // hold the arbiter off during the spin
}

// Emit one line of text to the VESC-Tool terminal (COMM_PRINT).
void VescTelemetry::send_print(const char *s)
{
    uint8_t buf[96];
    buf[0] = COMM_PRINT;
    uint16_t n = 0;
    while (s[n] != 0 && n < sizeof(buf) - 2) {
        buf[n + 1] = uint8_t(s[n]);
        n++;
    }
    send_packet(buf, uint16_t(n + 1));
}

void VescTelemetry::print_hall_table()
{
    float deg[8];
    _mc.get_hall_table_deg(deg);
    send_print(_mc.hall_table_valid()
                   ? "hall table (VALID):"
                   : "hall table (INVALID - run 'hall_detect' with the motor free to spin):");
    char line[48];
    for (uint8_t k = 0; k < 8; k++) {
        if (isnan(deg[k])) {
            hal.util->snprintf(line, sizeof(line), "  state %u: ---", k);
        } else {
            hal.util->snprintf(line, sizeof(line), "  state %u: %d deg", k, int(deg[k] + 0.5f));
        }
        send_print(line);
    }
}

// VESC-Tool terminal command (ascii payload after the id byte). Minimal set so
// the hall table can be read/triggered from VESC Tool without MCCONF support.
void VescTelemetry::handle_terminal()
{
    char cmd[48];
    uint16_t n = (_payload_len > 1) ? (_payload_len - 1) : 0;
    if (n >= sizeof(cmd)) n = sizeof(cmd) - 1;
    memcpy(cmd, &_payload[1], n);
    cmd[n] = 0;
    while (n > 0 && (cmd[n - 1] == '\n' || cmd[n - 1] == '\r' || cmd[n - 1] == ' ')) {
        cmd[--n] = 0;
    }

    if (strncmp(cmd, "hall_detect", 11) == 0) {
        _mc.start_hall_detect();
        _hall_detect_pending = true;
        _override_ms = AP_HAL::millis();
        send_print("hall detect started (~2 s); keep the motor free to spin, then run 'hall'");
    } else if (strncmp(cmd, "hall", 4) == 0) {
        print_hall_table();
    } else {
        send_print("commands: hall (show table) | hall_detect (run detection)");
    }
}

// Serialize a full VESC FW 6.00 mc_configuration so VESC Tool's "Read Motor
// Configuration" succeeds and populates its FOC → Hall Sensors tab. The field
// ORDER, TYPES and MCCONF_SIGNATURE must match confgenerator.c for FW 6.00
// exactly — VESC Tool deserializes sequentially, so any drift corrupts every
// field after it. Only the hall/identity fields carry our live values; the rest
// are plausible VESC defaults (they don't affect the hall read-out but must be
// present so the byte layout matches). Field numbers below index confgenerator.
void VescTelemetry::handle_get_mcconf(uint8_t reply_id)
{
    // ── Live values from the controller ────────────────────────────────────
    const float i_max = _mc.current_limit();
    float L, R, flux;
    _mc.get_motor_lrflux(L, R, flux);
    const float kp = _mc.get_current_kp();
    const float ki = (L > 1e-9f) ? (kp * R / L) : 0.0f;
    float bl_lo, bl_hi;
    _mc.get_hall_blend_erpm(bl_lo, bl_hi);
    const bool is_hall = (_mc.get_sensor_mode() == MotorControl::SensorMode::HALL);
    float hd[8];
    _mc.get_hall_table_deg(hd);   // [0..360) per state, NaN = unmapped

    uint8_t buf[512];
    uint8_t *p = buf;

    put_u8      (p, reply_id);                 // packet id (echoes request)
    put_u32     (p, MCCONF_SIGNATURE);         // 1

    put_u8      (p, 0);                        // 2  pwm_mode
    put_u8      (p, 0);                        // 3  comm_mode
    put_u8      (p, 2);                        // 4  motor_type = FOC
    put_u8      (p, 0);                        // 5  sensor_mode (BLDC)
    put_f32_auto(p, i_max);                    // 6  l_current_max
    put_f32_auto(p, -i_max);                   // 7  l_current_min
    put_f32_auto(p, 60.0f);                    // 8  l_in_current_max
    put_f32_auto(p, -60.0f);                   // 9  l_in_current_min
    put_f32_auto(p, 150.0f);                   // 10 l_abs_current_max
    put_f32_auto(p, -100000.0f);               // 11 l_min_erpm
    put_f32_auto(p, 100000.0f);                // 12 l_max_erpm
    put_f16     (p, 0.8f, 10000.0f);           // 13 l_erpm_start
    put_f32_auto(p, 300.0f);                   // 14 l_max_erpm_fbrake
    put_f32_auto(p, 1500.0f);                  // 15 l_max_erpm_fbrake_cc
    put_f32_auto(p, 6.0f);                     // 16 l_min_vin
    put_f32_auto(p, 57.0f);                    // 17 l_max_vin
    put_f32_auto(p, 10.0f);                    // 18 l_battery_cut_start
    put_f32_auto(p, 8.0f);                     // 19 l_battery_cut_end
    put_u8      (p, 1);                        // 20 l_slow_abs_current
    put_f16     (p, 85.0f, 10.0f);             // 21 l_temp_fet_start
    put_f16     (p, 105.0f, 10.0f);            // 22 l_temp_fet_end
    put_f16     (p, 85.0f, 10.0f);             // 23 l_temp_motor_start
    put_f16     (p, 105.0f, 10.0f);            // 24 l_temp_motor_end
    put_f16     (p, 0.15f, 10000.0f);          // 25 l_temp_accel_dec
    put_f16     (p, 0.005f, 10000.0f);         // 26 l_min_duty
    put_f16     (p, 0.95f, 10000.0f);          // 27 l_max_duty
    put_f32_auto(p, 500000.0f);                // 28 l_watt_max
    put_f32_auto(p, -500000.0f);               // 29 l_watt_min
    put_f16     (p, 1.0f, 10000.0f);           // 30 l_current_max_scale
    put_f16     (p, 1.0f, 10000.0f);           // 31 l_current_min_scale
    put_f16     (p, 1.0f, 10000.0f);           // 32 l_duty_start
    put_f32_auto(p, 150.0f);                   // 33 sl_min_erpm
    put_f32_auto(p, 1100.0f);                  // 34 sl_min_erpm_cycle_int_limit
    put_f32_auto(p, 10.0f);                    // 35 sl_max_fullbreak_current_dir_change
    put_f16     (p, 62.0f, 10.0f);             // 36 sl_cycle_int_limit
    put_f16     (p, 0.8f, 10000.0f);           // 37 sl_phase_advance_at_br
    put_f32_auto(p, 80000.0f);                 // 38 sl_cycle_int_rpm_br
    put_f32_auto(p, 600.0f);                    // 39 sl_bemf_coupling_k
    for (uint8_t k = 0; k < 8; k++) put_u8(p, 255); // 40-47 hall_table[0-7] (BLDC, unused)
    put_f32_auto(p, 2000.0f);                  // 48 hall_sl_erpm
    put_f32_auto(p, kp);                        // 49 foc_current_kp
    put_f32_auto(p, ki);                        // 50 foc_current_ki
    put_f32_auto(p, 20000.0f);                 // 51 foc_f_zv (switching freq)
    put_f32_auto(p, 0.36f);                    // 52 foc_dt_us
    put_u8      (p, 0);                        // 53 foc_encoder_inverted
    put_f32_auto(p, 0.0f);                     // 54 foc_encoder_offset
    put_f32_auto(p, 7.0f);                     // 55 foc_encoder_ratio
    put_u8      (p, is_hall ? 2 : 0);          // 56 foc_sensor_mode (2 = HALL) ← key
    put_f32_auto(p, 2000.0f);                  // 57 foc_pll_kp
    put_f32_auto(p, 30000.0f);                 // 58 foc_pll_ki
    put_f32_auto(p, L);                         // 59 foc_motor_l [H]
    put_f32_auto(p, 0.0f);                     // 60 foc_motor_ld_lq_diff
    put_f32_auto(p, R);                         // 61 foc_motor_r [Ω]
    put_f32_auto(p, flux);                      // 62 foc_motor_flux_linkage [Wb]
    put_f32_auto(p, 0.001f);                   // 63 foc_observer_gain
    put_f32_auto(p, 0.05f);                    // 64 foc_observer_gain_slow
    put_f16     (p, 0.0f, 1000.0f);            // 65 foc_observer_offset
    put_f32_auto(p, 10.0f);                    // 66 foc_duty_dowmramp_kp
    put_f32_auto(p, 200.0f);                   // 67 foc_duty_dowmramp_ki
    put_f16     (p, 1.0f, 10000.0f);           // 68 foc_start_curr_dec
    put_f32_auto(p, 2500.0f);                  // 69 foc_start_curr_dec_rpm
    put_f32_auto(p, 400.0f);                   // 70 foc_openloop_rpm
    put_f16     (p, 0.0f, 1000.0f);            // 71 foc_openloop_rpm_low
    put_f16     (p, 1.0f, 1000.0f);            // 72 foc_d_gain_scale_start
    put_f16     (p, 0.2f, 1000.0f);            // 73 foc_d_gain_scale_max_mod
    put_f16     (p, 0.1f, 100.0f);             // 74 foc_sl_openloop_hyst
    put_f16     (p, 0.0f, 100.0f);             // 75 foc_sl_openloop_time_lock
    put_f16     (p, 0.1f, 100.0f);             // 76 foc_sl_openloop_time_ramp
    put_f16     (p, 0.05f, 100.0f);            // 77 foc_sl_openloop_time
    put_f16     (p, 5.0f, 100.0f);             // 78 foc_sl_openloop_boost_q
    put_f16     (p, -1.0f, 100.0f);            // 79 foc_sl_openloop_max_q
    for (uint8_t k = 0; k < 8; k++) {          // 80-87 foc_hall_table[0-7] ← key
        if (isnan(hd[k])) {
            put_u8(p, 255);                    // unmapped state
        } else {
            float a = hd[k];
            while (a < 0.0f)    a += 360.0f;
            while (a >= 360.0f) a -= 360.0f;
            put_u8(p, uint8_t(lroundf(a * (200.0f / 360.0f)) % 200));
        }
    }
    put_f32_auto(p, 500.0f);                   // 88 foc_hall_interp_erpm
    put_f32_auto(p, bl_hi);                     // 89 foc_sl_erpm (hall→observer) ← relevant
    put_u8      (p, 0);                        // 90 foc_sample_v0_v7
    put_u8      (p, 0);                        // 91 foc_sample_high_current
    put_u8      (p, 0);                        // 92 foc_sat_comp_mode
    put_f16     (p, 0.0f, 1000.0f);            // 93 foc_sat_comp
    put_u8      (p, 0);                        // 94 foc_temp_comp
    put_f16     (p, 25.0f, 100.0f);            // 95 foc_temp_comp_base_temp
    put_f16     (p, 0.1f, 10000.0f);           // 96 foc_current_filter_const
    put_u8      (p, 0);                        // 97 foc_cc_decoupling
    put_u8      (p, 0);                        // 98 foc_observer_type
    put_f16     (p, 5.0f, 10.0f);              // 99 foc_hfi_voltage_start
    put_f16     (p, 2.0f, 10.0f);              // 100 foc_hfi_voltage_run
    put_f16     (p, 10.0f, 10.0f);             // 101 foc_hfi_voltage_max
    put_f16     (p, 1.0f, 1000.0f);            // 102 foc_hfi_gain
    put_f16     (p, 1.0f, 100.0f);             // 103 foc_hfi_hyst
    put_f32_auto(p, 2000.0f);                  // 104 foc_sl_erpm_hfi
    put_u16     (p, 65);                       // 105 foc_hfi_start_samples
    put_f32_auto(p, 0.001f);                   // 106 foc_hfi_obs_ovr_sec
    put_u8      (p, 0);                        // 107 foc_hfi_samples
    put_u8      (p, 1);                        // 108 foc_offsets_cal_on_boot
    for (uint8_t k = 0; k < 3; k++) put_f32_auto(p, 2048.0f);       // 109-111 foc_offsets_current
    for (uint8_t k = 0; k < 3; k++) put_f16(p, 0.0f, 10000.0f);     // 112-114 foc_offsets_voltage
    for (uint8_t k = 0; k < 3; k++) put_f16(p, 0.0f, 10000.0f);     // 115-117 foc_offsets_voltage_undriven
    put_u8      (p, 0);                        // 118 foc_phase_filter_enable
    put_u8      (p, 0);                        // 119 foc_phase_filter_disable_fault
    put_f32_auto(p, 4000.0f);                  // 120 foc_phase_filter_max_erpm
    put_u8      (p, 0);                        // 121 foc_mtpa_mode
    put_f32_auto(p, 0.0f);                     // 122 foc_fw_current_max
    put_f16     (p, 0.9f, 10000.0f);           // 123 foc_fw_duty_start
    put_f16     (p, 0.2f, 1000.0f);            // 124 foc_fw_ramp_time
    put_f16     (p, 0.02f, 10000.0f);          // 125 foc_fw_q_current_factor
    put_u8      (p, 0);                        // 126 foc_speed_soure
    put_i16     (p, 0);                        // 127 gpd_buffer_notify_left
    put_i16     (p, 0);                        // 128 gpd_buffer_interpol
    put_f16     (p, 0.1f, 10000.0f);           // 129 gpd_current_filter_const
    put_f32_auto(p, 0.03f);                    // 130 gpd_current_kp
    put_f32_auto(p, 50.0f);                    // 131 gpd_current_ki
    put_u8      (p, 0);                        // 132 sp_pid_loop_rate
    put_f32_auto(p, 0.004f);                   // 133 s_pid_kp
    put_f32_auto(p, 0.004f);                   // 134 s_pid_ki
    put_f32_auto(p, 0.0001f);                  // 135 s_pid_kd
    put_f16     (p, 0.2f, 10000.0f);           // 136 s_pid_kd_filter
    put_f32_auto(p, 900.0f);                   // 137 s_pid_min_erpm
    put_u8      (p, 1);                        // 138 s_pid_allow_braking
    put_f32_auto(p, 25000.0f);                 // 139 s_pid_ramp_erpms_s
    put_f32_auto(p, 0.03f);                    // 140 p_pid_kp
    put_f32_auto(p, 0.0f);                     // 141 p_pid_ki
    put_f32_auto(p, 0.0004f);                  // 142 p_pid_kd
    put_f32_auto(p, 0.0004f);                  // 143 p_pid_kd_proc
    put_f16     (p, 0.2f, 10000.0f);           // 144 p_pid_kd_filter
    put_f32_auto(p, 1.0f);                     // 145 p_pid_ang_div
    put_f16     (p, 0.0f, 10.0f);              // 146 p_pid_gain_dec_angle
    put_f32_auto(p, 0.0f);                     // 147 p_pid_offset
    put_f16     (p, 0.01f, 10000.0f);          // 148 cc_startup_boost_duty
    put_f32_auto(p, 0.0f);                     // 149 cc_min_current
    put_f32_auto(p, 0.0046f);                  // 150 cc_gain
    put_f16     (p, 0.04f, 10000.0f);          // 151 cc_ramp_step_max
    put_i32     (p, 500);                      // 152 m_fault_stop_time_ms
    put_f16     (p, 0.02f, 10000.0f);          // 153 m_duty_ramp_step
    put_f32_auto(p, 0.5f);                     // 154 m_current_backoff_gain
    put_u32     (p, 8192);                     // 155 m_encoder_counts
    put_f16     (p, 0.0f, 1000.0f);            // 156 m_encoder_sin_amp
    put_f16     (p, 0.0f, 1000.0f);            // 157 m_encoder_cos_amp
    put_f16     (p, 0.0f, 1000.0f);            // 158 m_encoder_sin_offset
    put_f16     (p, 0.0f, 1000.0f);            // 159 m_encoder_cos_offset
    put_f16     (p, 0.5f, 1000.0f);            // 160 m_encoder_sincos_filter_constant
    put_f16     (p, 0.0f, 1000.0f);            // 161 m_encoder_sincos_phase_correction
    put_u8      (p, 0);                        // 162 m_sensor_port_mode
    put_u8      (p, 0);                        // 163 m_invert_direction
    put_u8      (p, 0);                        // 164 m_drv8301_oc_mode
    put_u8      (p, 16);                       // 165 m_drv8301_oc_adj
    put_f32_auto(p, 3000.0f);                  // 166 m_bldc_f_sw_min
    put_f32_auto(p, 30000.0f);                 // 167 m_bldc_f_sw_max
    put_f32_auto(p, 25000.0f);                 // 168 m_dc_f_sw
    put_f32_auto(p, 3380.0f);                  // 169 m_ntc_motor_beta
    put_u8      (p, 0);                        // 170 m_out_aux_mode
    put_u8      (p, 0);                        // 171 m_motor_temp_sens_type
    put_f32_auto(p, 0.61f);                    // 172 m_ptc_motor_coeff
    put_f16     (p, 10000.0f, 0.1f);           // 173 m_ntcx_ptcx_res
    put_f16     (p, 25.0f, 10.0f);             // 174 m_ntcx_ptcx_temp_base
    put_u8      (p, 0);                        // 175 m_hall_extra_samples
    put_u8      (p, 10);                       // 176 m_batt_filter_const
    put_u8      (p, uint8_t(_pole_pairs * 2)); // 177 si_motor_poles ← RPM display
    put_f32_auto(p, 1.0f);                     // 178 si_gear_ratio
    put_f32_auto(p, 0.083f);                   // 179 si_wheel_diameter
    put_u8      (p, 0);                        // 180 si_battery_type
    put_u8      (p, 3);                        // 181 si_battery_cells
    put_f32_auto(p, 0.0f);                     // 182 si_battery_ah
    put_f32_auto(p, 0.0f);                     // 183 si_motor_nl_current
    put_u8      (p, 0);                        // 184 bms.type
    put_u8      (p, 0);                        // 185 bms.limit_mode
    put_f16     (p, 0.0f, 100.0f);             // 186 bms.t_limit_start
    put_f16     (p, 0.0f, 100.0f);             // 187 bms.t_limit_end
    put_f16     (p, 0.0f, 1000.0f);            // 188 bms.soc_limit_start
    put_f16     (p, 0.0f, 1000.0f);            // 189 bms.soc_limit_end
    put_u8      (p, 0);                        // 190 bms.fwd_can_mode

    send_packet(buf, uint16_t(p - buf));
}

} // namespace ChibiOS

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
