#include "VescTelemetry.h"
#include "MotorControl.h"
#include "stm32_foc_motor_control.h"

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <hal.h>
#include <string.h>
#include <stdlib.h>
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
// ─── FOC_WATCH ─── temporary diagnostic, remove as a unit (see MotorControl.h)
#if FOC_WATCH
constexpr uint32_t WATCH_HZ = 20;           // `watch` stream rate
#endif
// ─── end FOC_WATCH ───────────────────────────────────────────────────────────
constexpr uint8_t COMM_PRINT        = 21;   // terminal text response
// Real VESC COMM_PACKET_ID value (datatypes.h: FW_VERSION=0 … SAMPLE_PRINT=19,
// TERMINAL_CMD=20, PRINT=21, ROTOR_POSITION=22, EXPERIMENT_SAMPLE=23,
// DETECT_MOTOR_PARAM=24, DETECT_MOTOR_R_L=25, DETECT_MOTOR_FLUX_LINKAGE=26,
// DETECT_ENCODER=27, DETECT_HALL_FOC=28). This is what VESC Tool's hall detect
// button sends; must match exactly or the command is silently dropped.
constexpr uint8_t COMM_DETECT_HALL_FOC = 28;
// Give-up window for a hall-detect spin that never reports a result. The spin
// itself is HD_RAMP_S + 2·HD_REVS/HD_HZ ≈ 12 s (MotorControl.cpp), so this is
// ~2.5× the healthy duration: long enough that it can only fire on a run that
// genuinely died, short enough that the flag does not latch for the session.
constexpr uint32_t HALL_DETECT_TIMEOUT_MS = 30000;
// Parameter detection. VESC Tool's FOC tab sends R_L for "Measure R/L" and
// FLUX_LINKAGE_OPENLOOP (57, NOT the legacy 26) for "Measure λ"; the motor
// wizard sends APPLY_ALL_FOC. All three are "blocking commands" in VESC —
// answered from a worker thread when the measurement completes, so the tool
// tolerates a reply seconds later (comm/commands.c blocking-command list).
constexpr uint8_t COMM_DETECT_MOTOR_R_L                 = 25;
constexpr uint8_t COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP = 57;
constexpr uint8_t COMM_DETECT_APPLY_ALL_FOC             = 58;
constexpr uint8_t COMM_SET_MCCONF         = 13;  // VESC Tool "Write Motor Configuration"
constexpr uint8_t COMM_GET_MCCONF         = 14;  // VESC Tool "Read Motor Configuration"
constexpr uint8_t COMM_GET_MCCONF_DEFAULT = 15;
constexpr uint8_t COMM_SET_APPCONF         = 16;  // VESC Tool "Write App Configuration"
constexpr uint8_t COMM_GET_APPCONF         = 17;  // VESC Tool "Read App Configuration"
constexpr uint8_t COMM_GET_APPCONF_DEFAULT = 18;

constexpr uint8_t FW_MAJOR = 6;
constexpr uint8_t FW_MINOR = 6;
// mc_configuration serialization signature for VESC FW 6.06 (confgenerator.h
// MCCONF_SIGNATURE). VESC Tool validates this before parsing the blob, so it is
// version-locked: it must match the FW version reported by COMM_FW_VERSION (6.6)
// AND the config layout emitted by handle_get_mcconf()/handle_set_mcconf() below,
// field-for-field (order + type/scale) against release_6_06/confgenerator.c.
constexpr uint32_t MCCONF_SIGNATURE = 788332866u;
// app_configuration signature, same version lock as MCCONF_SIGNATURE above and
// the same consequence if it drifts: VESC Tool refuses the blob outright. Both
// are CRC32C over VESC Tool's own parameter definition for the version — for
// 6.06 that is res/config/6.06/parameters_appconf.xml in the vesc_tool tree,
// hashed as name+type+vTx+enum-names across the <SerOrder> list
// (ConfigParams::getSignature). Recomputing the mcconf one the same way returns
// 788332866, which is how this value was checked rather than guessed.
constexpr uint32_t APPCONF_SIGNATURE = 2099347128u;
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

// ── Unpack helpers (big-endian, advance p) — inverse of the put_* packers ────
inline uint16_t get_u16(const uint8_t *&p) {
    uint16_t v = (uint16_t(p[0]) << 8) | p[1];
    p += 2;
    return v;
}
inline uint32_t get_u32(const uint8_t *&p) {
    uint32_t v = (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) |
                 (uint32_t(p[2]) << 8)  | p[3];
    p += 4;
    return v;
}
inline float get_f16(const uint8_t *&p, float scale) {
    return float(int16_t(get_u16(p))) / scale;
}
// VESC buffer_get_float32(): fixed-point int32 divided by a per-field scale.
// Distinct from get_f32_auto() (buffer_get_float32_auto), which is a packed
// exponent/mantissa form — the detection commands use the SCALED variant.
inline float get_f32(const uint8_t *&p, float scale) {
    return float(int32_t(get_u32(p))) / scale;
}
// Inverse of put_f32_auto (VESC buffer_get_float32_auto).
inline float get_f32_auto(const uint8_t *&p) {
    uint32_t res = get_u32(p);
    int      e     = (res >> 23) & 0xFF;
    uint32_t sig_i = res & 0x7FFFFF;
    bool     neg   = (res & (1u << 31)) != 0;
    float    sig   = 0.0f;
    if (e != 0) {
        sig = float(sig_i) / (8388608.0f * 2.0f) + 0.5f;
        e  -= 126;
    }
    float ret = ldexpf(sig, e);
    return neg ? -ret : ret;
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
        // VESC Tool over USB CDC ignores baud, but bigger buffers matter: the
        // GET_VALUES reply is ~80 B at 50–100 Hz, and a COMM_SET_MCCONF frame is
        // ~470 B — the RX ring must hold a whole one so no bytes are dropped
        // before update() drains it.
        _uart->begin(115200, 1024, 512);
    }
}

void VescTelemetry::update()
{
    if (_uart == nullptr) {
        return;
    }
// ─── FOC_WATCH ─── temporary diagnostic, remove as a unit (see MotorControl.h)
#if FOC_WATCH
    if (_watch_until_ms != 0) {
        const uint32_t now = AP_HAL::millis();
        if (int32_t(now - _watch_until_ms) >= 0) {
            _watch_until_ms = 0;
            send_print("watch off");
        } else if (int32_t(now - _watch_next_ms) >= 0) {
            _watch_next_ms = now + (1000U / WATCH_HZ);
            print_watch();
        }
    }
#endif
// ─── end FOC_WATCH ───────────────────────────────────────────────────────────
    // Hall-table detection runs asynchronously (a ~12 s current-controlled spin in
    // the ISR — HD_RAMP_S + 2·HD_REVS/HD_HZ). The arbiter stands off for the whole
    // spin on MotorControl::is_hall_detecting(); the _override_ms refresh here is
    // belt-and-braces for the same purpose. Emit the result once it lands, in VESC
    // Tool's native COMM_DETECT_HALL_FOC reply format:
    //   [id, hall_tab[0..7] uint8 (angle·200/360; 255 = invalid), res (0 = ok)].
    if (_hall_detect_pending) {
        _override_ms = AP_HAL::millis();
        float deg[8];
        // Give up if the result never arrives — start_hall_detect() refuses outright
        // while a trip is latched, and a fault mid-spin coasts it without ever
        // setting done. Without this the flag latches true for the rest of the
        // session: no reply is ever sent, the next detect looks identical from the
        // host, and diag reports a hall_pending that no longer means anything is
        // running. Timed generously against the spin length so a healthy run can
        // never trip it.
        if (_hall_detect_ms != 0 &&
            (AP_HAL::millis() - _hall_detect_ms) > HALL_DETECT_TIMEOUT_MS) {
            _hall_detect_pending = false;
            _hall_detect_ms      = 0;
            _override_ms         = 0;   // stop holding the arbiter off
            send_print("hall detect: timed out with no result (bridge faulted or never started)");
        } else if (_mc.hall_detect_result(deg)) {
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
            _hall_detect_ms      = 0;
        }
    }

    uint32_t avail = _uart->available();
    // Cap per-call work to avoid starving the periph loop, but large enough to
    // drain a whole COMM_SET_MCCONF frame (~477 B) in one service call.
    if (avail > 512) avail = 512;
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
        } else if (b == 0x03) {
            // Long frame (2-byte length). Needed for COMM_SET_MCCONF, whose
            // ~470-byte payload can't fit a short (0x02) frame. A stray 0x03
            // (also the end byte) that mis-latches here just fails the length
            // bound or the final CRC check and resyncs within one frame.
            _state = RxState::WAIT_LEN_LONG_HI;
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
    case COMM_DETECT_MOTOR_R_L:
    case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP:
    case COMM_DETECT_APPLY_ALL_FOC:
        handle_detect(_payload[0]);
        break;
    case COMM_GET_MCCONF:
    case COMM_GET_MCCONF_DEFAULT:
        // VESC Tool reads the motor config (incl. the FOC hall table) via this.
        // Reply id echoes the request so the tool routes it correctly.
        handle_get_mcconf(_payload[0]);
        break;
    case COMM_SET_MCCONF:
        // VESC Tool "Write Motor Configuration": persist the standard fields we
        // back with params, then the sink reboots to apply.
        handle_set_mcconf();
        break;
    case COMM_GET_APPCONF:
    case COMM_GET_APPCONF_DEFAULT:
        // App Settings → PPM. Without a reply here the tool reports a read
        // failure and never opens the page the control type lives on.
        handle_get_appconf(_payload[0]);
        break;
    case COMM_SET_APPCONF:
        handle_set_appconf();
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
    put_u8 (p, _mc.reported_fault());   // fault_code (held briefly after clearing)
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
    // A current command REPLACES any rpm/duty/brake bench override — both because
    // that is VESC's semantic (a new setpoint selects the control mode) and
    // because VESC Tool's STOP button is a COMM_SET_CURRENT of 0. While an
    // override is active the arbiter deliberately stands off and applies nothing,
    // so without this the stop button cannot reach the motor and a speed-mode
    // spin cannot be commanded down from the tool at all.
    _override_ms = 0;
    // Belt and braces on the stop path: drive the release straight into the
    // controller as well, so stopping never depends on the arbiter running or on
    // which source it happens to pick this cycle.
    if (ma == 0) {
        _mc.set_current(0.0f);
    }
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
    // Zero release drops the override so CAN/PWM arbitration resumes.
    _override_ms = (ma != 0) ? AP_HAL::millis() : 0;
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
    _override_ms = (d != 0) ? AP_HAL::millis() : 0;
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
    _override_ms = (erpm != 0) ? AP_HAL::millis() : 0;
}

// COMM_DETECT_HALL_FOC: start a hall-table detection spin (payload current arg
// ignored — the firmware uses its own fixed low-modulation open-loop spin). The
// result is emitted asynchronously from update() when the spin completes.
void VescTelemetry::handle_detect_hall()
{
    // COMM_DETECT_HALL_FOC carries the detection current as an int32 in
    // milliamps — VESC commands.c:2245, buffer_get_float32(data, 1e3). It is
    // what VESC Tool's hall-detection box sends, so ignoring it left that box
    // doing nothing and the spin pinned to the built-in default. An absent or
    // nonsensical field falls back to that default rather than to zero current.
    float amps = 0.0f;
    if (_payload_len >= 5) {
        const uint8_t *p = &_payload[1];
        amps = get_f32(p, 1000.0f);
        if (!(amps > 0.0f)) {
            amps = 0.0f;
        }
    }
    _mc.start_hall_detect(amps);
    _hall_detect_pending = true;
    _hall_detect_ms      = AP_HAL::millis();
    _override_ms         = AP_HAL::millis();   // hold the arbiter off during the spin
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
    char line[64];
    for (uint8_t k = 0; k < 8; k++) {
        if (isnan(deg[k])) {
            hal.util->snprintf(line, sizeof(line), "  state %u: ---", k);
        } else {
            hal.util->snprintf(line, sizeof(line), "  state %u: %d deg",
                               k, int(deg[k] + 0.5f));
        }
        send_print(line);
    }
}

// Dump every input the throttle arbiter decides on. The arbiter picks, in
// order: chime → override → CAN → USB → PWM → coast, and a motor that sits in
// IDLE with no fault means one of the earlier branches is winning or the whole
// block is being skipped. Nothing here is inferable from COMM_GET_VALUES, which
// is why a stuck arbiter looks identical to "no torque" from the outside.
void VescTelemetry::print_diag()
{
    const uint32_t now = AP_HAL::millis();
    char line[96];
    float amps = 0.0f;
    const bool usb_ok = usb_current(now, 200, amps);
    const bool ovr_ok = override_active(now, 200);

    hal.util->snprintf(line, sizeof(line), "init=%u state=%u fault=%u zero=%u vbus_rdy=%u",
                       unsigned(_mc.is_initialized()), unsigned(_mc.get_state()),
                       unsigned(_mc.get_fault()), unsigned(_mc.zero_valid()),
                       unsigned(_mc.vbus_ready()));
    send_print(line);
    hal.util->snprintf(line, sizeof(line), "usb_fresh=%u amps=%.2f set_ms=%lu age=%lu",
                       unsigned(usb_ok), double(amps),
                       (unsigned long)_usb_current_ms,
                       (unsigned long)(now - _host_alive_ms));
    send_print(line);
    hal.util->snprintf(line, sizeof(line), "override=%u ovr_ms=%lu hall_pending=%u",
                       unsigned(ovr_ok), (unsigned long)_override_ms,
                       unsigned(_hall_detect_pending));
    send_print(line);
    // The silent refusal paths. A locked-out controller reports fault=0 and
    // sits in IDLE, indistinguishable from "makes no torque" without this.
    hal.util->snprintf(line, sizeof(line), "sensor=%s ol_lock=%u ol_try=%u hall_tab=%u",
                       (_mc.get_sensor_mode() == MotorControl::SensorMode::HALL) ? "HALL" : "SNSRLESS",
                       unsigned(_mc.ol_locked_out()), unsigned(_mc.ol_attempts()),
                       unsigned(_mc.hall_table_valid()));
    send_print(line);
    // App-config write forensics. rx=0 means no SET_APPCONF frame ever reached
    // this handler; badsig means the blob was for a different FW layout; ctrl is
    // what came out of the parse (255 = nothing parsed yet). run_ctrl is what the
    // decode is actually using, read from the param at boot — if ctrl matches the
    // value written and run_ctrl still does not, the refusal is downstream, in
    // the param store rather than the wire.
    hal.util->snprintf(line, sizeof(line),
                       "appconf rx=%u sig_bad=%u short=%u ctrl=%u saved=%u live=%u",
                       unsigned(_appconf_rx), unsigned(_appconf_badsig),
                       unsigned(_appconf_short), unsigned(_appconf_ctrl),
                       unsigned(_appconf_ok), unsigned(_conf.ppm_ctrl_type));
    send_print(line);
    if (_storage_full) {
        send_print("PARAM STORAGE FULL - set_and_save() is a no-op, nothing persists");
    }
    // Bus extremes as the TRIPS saw them — both are raw samples, so neither
    // matches the filtered vbus reported in telemetry. vmax is the number that
    // explains an over-voltage trip, which on regen can overshoot and fall back
    // faster than any host-rate readout can catch.
    hal.util->snprintf(line, sizeof(line), "vbus=%.1f vmin=%.1f vmax=%.1f",
                       double(_mc.get_vbus()), double(_mc.get_vbus_min_seen()),
                       double(_mc.get_vbus_max_seen()));
    send_print(line);
    // Bridge hardware vs what the driver believes. `on` is MotorControl's
    // _outputs_on; `moe` is the live TIM1 BDTR bit. on=1 moe=0 means the bridge
    // is coasting while arm_bridge() short-circuits on the stale flag — no
    // differential reaches the motor and nothing upstream reports a fault.
    // The CCRs show whether the duties written are actually differential:
    // all three equal = common mode only = no torque whatever MOE says.
    uint16_t ccr_u = 0, ccr_v = 0, ccr_w = 0;
    stm32_foc_motor_control_read_ccr(ccr_u, ccr_v, ccr_w);
    hal.util->snprintf(line, sizeof(line), "on=%u moe=%u ccr=%u/%u/%u top=%u",
                       unsigned(_mc.outputs_on()),
                       unsigned(stm32_foc_motor_control_moe_set()),
                       unsigned(ccr_u), unsigned(ccr_v), unsigned(ccr_w),
                       unsigned(_mc.period_ticks()));
    send_print(line);
}

// ── Parameter detection: request parsing and replies ────────────────────────
// Field order, types and SCALES are fixed by comm/commands.c; a mismatch is
// silently misread rather than rejected, so they are spelled out per field.
void VescTelemetry::handle_detect(uint8_t id)
{
    if (_detect_cb == nullptr) {
        return;
    }
    DetectReq req{};
    const uint8_t *p = &_payload[1];
    const uint16_t n = (_payload_len > 1) ? uint16_t(_payload_len - 1) : 0;

    if (id == COMM_DETECT_MOTOR_R_L) {
        req.kind = DetectKind::R_L;   // no payload
    } else if (id == COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP) {
        if (n < 16) {
            return;
        }
        req.kind         = DetectKind::FLUX_OPENLOOP;
        req.current      = get_f32(p, 1e3f);
        req.erpm_per_sec = get_f32(p, 1e3f);
        req.duty         = get_f32(p, 1e3f);
        req.resistance   = get_f32(p, 1e6f);
        // Inductance is optional — older VESC Tool builds omit it, and
        // conf_general_measure_flux_linkage_openloop() then falls back to the
        // configured value. Mirror that rather than reading past the payload.
        if (n >= 20) {
            req.inductance = get_f32(p, 1e8f);
        }
    } else {   // COMM_DETECT_APPLY_ALL_FOC
        if (n < 21) {
            return;
        }
        req.kind = DetectKind::APPLY_ALL_FOC;
        p += 1;                                  // detect_can — single-ESC build
        req.max_power_loss = get_f32(p, 1e3f);
        p += 4; p += 4;                          // min/max_current_in — unused here
        req.openloop_erpm  = get_f32(p, 1e3f);
        req.sl_erpm        = get_f32(p, 1e3f);
    }
    _override_ms = AP_HAL::millis();   // bench override: the throttle arbiter stands off
    _detect_cb(_detect_ctx, req);
}

void VescTelemetry::send_detect_r_l(float r, float l, float ld_lq_diff)
{
    uint8_t buf[16];
    uint8_t *p = buf;
    *p++ = COMM_DETECT_MOTOR_R_L;
    put_f32(p, r,           1e6f);   // [Ω]
    put_f32(p, l,           1e3f);   // VESC reports inductance in MICROhenry here
    put_f32(p, ld_lq_diff,  1e3f);   // [µH] — 0, we do not measure saliency
    send_packet(buf, uint16_t(p - buf));
}

void VescTelemetry::send_detect_flux(float linkage)
{
    uint8_t buf[20];
    uint8_t *p = buf;
    *p++ = COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP;
    put_f32(p, linkage, 1e7f);   // [Wb]; VESC also uses <0 values as error codes
    put_f32(p, -1.0f,   1e6f);   // enc_offset — no encoder
    put_f32(p, -1.0f,   1e6f);   // enc_ratio
    *p++ = 0;                    // enc_inverted
    send_packet(buf, uint16_t(p - buf));
}

void VescTelemetry::send_detect_apply_all(int16_t result)
{
    uint8_t buf[8];
    uint8_t *p = buf;
    *p++ = COMM_DETECT_APPLY_ALL_FOC;
    put_i16(p, result);   // >0 = ok (VESC returns the number of motors detected)
    send_packet(buf, uint16_t(p - buf));
}

// VESC-Tool terminal command (ascii payload after the id byte). Minimal set so
// the hall table can be read/triggered from VESC Tool without MCCONF support.
// See header. Angles are printed in degrees; the pairs that matter are
// theta-vs-hall (angle divergence) and iq-vs-iq_cmd (did the loop hold).
void VescTelemetry::print_trip()
{
    const MotorControl::FaultSnapshot &t = _mc.get_fault_snapshot();
    char line[110];
    if (!t.valid) {
        send_print("no trip recorded since boot");
        return;
    }
    const float r2d = 57.2957795f;
    hal.util->snprintf(line, sizeof(line), "trip fault=%u state=%u mode=%u vbus=%.1f",
                       unsigned(t.code), unsigned(t.state), unsigned(t.mode), double(t.vbus));
    send_print(line);
    // A residual far from zero means the three phases don't sum to zero, i.e. the
    // SENSE chain is lying — the trip is then an artifact, not a real current.
    hal.util->snprintf(line, sizeof(line), "  ia=%.1f ib=%.1f ic=%.1f resid=%.2f",
                       double(t.ia), double(t.ib), double(t.ic), double(t.i_resid));
    send_print(line);
    // theta vs hall: how far the angle actually commutated with had drifted from
    // the halls. hall_erpm 0 with pll_erpm nonzero = transitions had stopped and
    // the angle was being extrapolated.
    hal.util->snprintf(line, sizeof(line), "  theta=%.0f hall=%.0f obs=%.0f hstate=%u",
                       double(t.theta * r2d), double(t.hall_theta * r2d),
                       double(t.obs_theta * r2d), unsigned(t.hall_state));
    send_print(line);
    // Sign of pll_erpm says which side of the zero crossing the trip landed on.
    // hall = last observed sector rate, eff = the decayed value the logic acts on,
    // hticks = samples since the last edge. glitch counts sectors timed at under a
    // quarter of the previous one, i.e. spikes that survived the majority vote —
    // nonzero means the hall wiring or filtering is still passing noise.
    hal.util->snprintf(line, sizeof(line), "  pll=%.0f hall=%.0f eff=%.0f hticks=%lu glitch=%u",
                       double(t.pll_erpm), double(t.hall_erpm), double(t.hall_eff),
                       (unsigned long)t.hall_ticks, unsigned(t.hall_glitch));
    send_print(line);
    // iq far from iq_cmd = the current loop had lost control (angle error), as
    // opposed to iq tracking a reference that was simply too big.
    hal.util->snprintf(line, sizeof(line), "  iq_cmd=%.2f iq=%.2f id=%.2f bounds=[%.1f,%.1f]",
                       double(t.iq_cmd), double(t.iq), double(t.id),
                       double(t.iq_lo), double(t.iq_hi));
    send_print(line);
    // The loop's own output, for a dq current nobody commanded. Read vd against
    // the SIGN of id above: opposing = the loop is fighting the current and
    // losing, driving = the loop made it. int_d is the integrator alone, so
    // int_d near +/-vmax is windup. duty near 1.0 means the vector was clamped,
    // in which case vd/vq are the clamp's output and not the PI's intent.
    hal.util->snprintf(line, sizeof(line),
                       "  vd=%.2f vq=%.2f int_d=%.2f int_q=%.2f duty=%.2f vmax=%.1f",
                       double(t.vd), double(t.vq), double(t.integ_d),
                       double(t.integ_q), double(t.duty), double(t.v_max));
    send_print(line);
}

// ─── FOC_WATCH ─── temporary diagnostic, remove as a unit (see MotorControl.h)
#if FOC_WATCH
// One sample line. Angles in degrees; t is the angle actually commutated with,
// h the hall angle, o the observer's. o-h is the disagreement being chased, and
// its SIGN against the sign of e is the whole point of the exercise.
void VescTelemetry::print_watch()
{
    char line[128];
    const float r2d = 57.2957795f;
    float id = 0.0f, iq = 0.0f, vd = 0.0f, vq = 0.0f;
    _mc.get_idq(id, iq);
    // vd/vq are what lets lambda be back-solved OFF a trip: at constant speed
    // di/dt ~ 0, so E = V - I*Z holds and |E|/omega is the flux linkage with no
    // transient term to argue about. Mid-trip the L*di/dt term is the same size
    // as the d-axis result, so only a steady cruise settles it.
    _mc.get_vdq(vd, vq);
    const float h = _mc.get_hall_angle() * r2d;
    const float o = _mc.get_observer_angle() * r2d;
    float dif = o - h;
    while (dif > 180.0f)  { dif -= 360.0f; }
    while (dif < -180.0f) { dif += 360.0f; }
    hal.util->snprintf(line, sizeof(line),
                       "w t=%.0f h=%.0f o=%.0f d=%.0f e=%.0f he=%.0f id=%.1f iq=%.1f "
                       "vd=%.2f vq=%.2f s=%u",
                       double(_mc.get_estimated_angle() * r2d), double(h), double(o), double(dif),
                       double(_mc.get_erpm()), double(_mc.get_hall_erpm()),
                       double(id), double(iq), double(vd), double(vq),
                       unsigned(_mc.get_hall_state()));
    send_print(line);
}
#endif
// ─── end FOC_WATCH ───────────────────────────────────────────────────────────

void VescTelemetry::handle_terminal()
{
    char cmd[48];
    char line[96];
    uint16_t n = (_payload_len > 1) ? (_payload_len - 1) : 0;
    if (n >= sizeof(cmd)) n = sizeof(cmd) - 1;
    memcpy(cmd, &_payload[1], n);
    cmd[n] = 0;
    while (n > 0 && (cmd[n - 1] == '\n' || cmd[n - 1] == '\r' || cmd[n - 1] == ' ')) {
        cmd[--n] = 0;
    }

    if (strncmp(cmd, "hall_detect", 11) == 0) {
        // Optional current argument: "hall_detect [amps]". Same knob as the wire
        // command, because the terminal is how this is actually triggered here.
        const float det_a = strtof(cmd + 11, nullptr);
        _mc.start_hall_detect((det_a > 0.0f) ? det_a : 0.0f);
        _hall_detect_pending = true;
        _hall_detect_ms      = AP_HAL::millis();
        _override_ms         = AP_HAL::millis();
        hal.util->snprintf(line, sizeof(line),
                           "hall detect started at %.1f A (~12 s); keep the motor free to spin, then run 'hall'",
                           double(_mc.hall_detect_current()));
        send_print(line);
    } else if (strncmp(cmd, "hall", 4) == 0) {
        print_hall_table();
    } else if (strncmp(cmd, "diag", 4) == 0) {
        print_diag();
    } else if (strncmp(cmd, "trip", 4) == 0) {
        print_trip();
// ─── FOC_WATCH ─── temporary diagnostic, remove as a unit (see MotorControl.h)
#if FOC_WATCH
    } else if (strncmp(cmd, "watch", 5) == 0) {
        // "watch [seconds]": no argument = 10 s, an argument of 0 (or less)
        // stops it, anything else runs for that many seconds. Capped so a
        // forgotten stream can never sit on the link indefinitely.
        const char *arg = cmd + 5;
        while (*arg == ' ') { arg++; }
        const bool has_arg = (*arg != 0);
        long secs = has_arg ? strtol(arg, nullptr, 10) : 10;
        if (has_arg && secs <= 0) {
            _watch_until_ms = 0;
            send_print("watch off");
        } else {
            if (secs > 120) { secs = 120; }
            _watch_until_ms = AP_HAL::millis() + uint32_t(secs) * 1000U;
            _watch_next_ms  = AP_HAL::millis();
            hal.util->snprintf(line, sizeof(line), "watch on for %ld s at %u Hz",
                               secs, unsigned(WATCH_HZ));
            send_print(line);
        }
#endif
// ─── end FOC_WATCH ───────────────────────────────────────────────────────────
    } else if (strncmp(cmd, "beep", 4) == 0) {
        // On-demand tone: "beep [freq_hz] [amp] [ms]". The power-on chime is the
        // only thing known to push current through the windings on a board that
        // otherwise looks dead, but it fires before USB enumerates, so it can
        // never be observed over this link. Triggering it on demand makes that
        // current measurable while a telemetry poll is already running.
        const char *a = cmd + 4;
        char *end = nullptr;
        float   freq = strtof(a, &end);
        float   amp  = (end != nullptr) ? strtof(end, &end) : 0.0f;
        long    ms   = (end != nullptr) ? strtol(end, &end, 10) : 0;
        if (!(freq >= 200.0f))  { freq = 2000.0f; }
        if (!(amp  >  0.0f))    { amp  = 0.06f; }
        if (ms <= 0 || ms > 5000) { ms = 500; }
        _mc.play_tone(freq, amp, uint16_t(ms));
        _override_ms = AP_HAL::millis();   // bench override: arbiter stands off
        hal.util->snprintf(line, sizeof(line), "beep %.0f Hz amp %.3f for %ld ms",
                           double(freq), double(amp), ms);
        send_print(line);
    } else {
        send_print("commands: hall | hall_detect [A] | diag | trip | watch [s] | beep [hz] [amp] [ms]");
    }
}

// Serialize a full VESC FW 6.06 mc_configuration so VESC Tool's "Read Motor
// Configuration" succeeds and populates its FOC tabs. The field ORDER, TYPES
// and MCCONF_SIGNATURE must match release_6_06/confgenerator.c exactly — VESC
// Tool deserializes sequentially, so any drift corrupts every field after it.
// Fields we back with params carry live values (from MotorControl + the config
// snapshot); the rest are plausible VESC defaults (they must be present so the
// byte layout matches). Comment numbers index confgenerator.c fields.
void VescTelemetry::handle_get_mcconf(uint8_t reply_id)
{
    // ── Live / param-derived values ─────────────────────────────────────────
    const float i_max = _mc.current_limit();
    float L, R, flux;
    _mc.get_motor_lrflux(L, R, flux);
    const float kp = _mc.get_current_kp();
    const float ki = _mc.get_current_ki();
    float bl_lo, bl_hi;
    _mc.get_hall_blend_erpm(bl_lo, bl_hi);
    const bool is_hall = (_mc.get_sensor_mode() == MotorControl::SensorMode::HALL);
    float hd[8];
    _mc.get_hall_table_deg(hd);   // [0..360) per state, NaN = unmapped
    float ol_boost, ol_maxq, ol_erpm, ol_rpmlow, ol_hyst, ol_tlock, ol_tramp, ol_tconst;
    _mc.get_openloop_cfg(ol_boost, ol_maxq, ol_erpm, ol_rpmlow, ol_hyst,
                         ol_tlock, ol_tramp, ol_tconst);
    const float v_min  = _mc.get_vbus_min();
    const float uv_band = _mc.get_vbus_uv_band();

    uint8_t buf[512];
    uint8_t *p = buf;

    put_u8      (p, reply_id);                 // packet id (echoes request)
    put_u32     (p, MCCONF_SIGNATURE);

    put_u8      (p, 0);                        // pwm_mode
    put_u8      (p, 0);                        // comm_mode
    put_u8      (p, 2);                        // motor_type = FOC
    put_u8      (p, 0);                        // sensor_mode (BLDC)
    put_f32_auto(p, i_max);                    // l_current_max        ← param (motoring)
    put_f32_auto(p, -_mc.regen_limit());       // l_current_min        ← param (braking)
    put_f32_auto(p, 60.0f);                    // l_in_current_max
    put_f32_auto(p, -60.0f);                   // l_in_current_min
    put_f16     (p, 0.5f, 10000.0f);           // l_in_current_map_start
    put_f16     (p, 0.02f, 10000.0f);          // l_in_current_map_filter
    put_f32_auto(p, _conf.abs_current_max);    // l_abs_current_max    ← param (hard OC)
    put_f32_auto(p, _mc.get_min_erpm());       // l_min_erpm      ← param (MIN_ERPM)
    put_f32_auto(p, _mc.get_max_erpm());       // l_max_erpm      ← param (MAX_ERPM)
    put_f16     (p, _mc.get_erpm_start(), 10000.0f); // l_erpm_start ← param (ERPM_START)
    put_f32_auto(p, 300.0f);                   // l_max_erpm_fbrake
    put_f32_auto(p, 1500.0f);                  // l_max_erpm_fbrake_cc
    put_f16     (p, v_min, 10.0f);             // l_min_vin            ← param (V_MIN)
    put_f16     (p, _conf.max_vin, 10.0f);     // l_max_vin            ← param (V_OV, hard trip)
    // VESC's under-voltage foldback band, the mirror of the regen pair below:
    // motoring current scales to zero between start and end. Ours is V_MIN plus
    // the V_UVFOLD band, so end = V_MIN (where the hard UV trip also sits, which
    // is exactly how VESC configures it — see MotorControl.h vbus_uv_fold_band).
    put_f16     (p, v_min + uv_band, 10.0f);   // l_battery_cut_start  ← V_MIN + V_UVFOLD
    put_f16     (p, v_min, 10.0f);             // l_battery_cut_end    ← V_MIN
    // VESC's regen over-voltage cutoff (mc_interface.c:2483) — the fields that
    // actually mean "fold braking back as the bus rises", so V_MAX/V_FOLD live
    // here rather than in l_max_vin. Previously hardcoded 100/110 V, which showed
    // a nonsense band in VESC Tool while the real behaviour came from elsewhere.
    put_f16     (p, _conf.regen_cut_start, 10.0f); // l_battery_regen_cut_start ← V_MAX - V_FOLD
    put_f16     (p, _conf.regen_cut_end, 10.0f);   // l_battery_regen_cut_end   ← V_MAX
    put_u8      (p, 1);                        // l_slow_abs_current
    put_u8      (p, uint8_t(_conf.temp_fet_start)); // l_temp_fet_start ← param
    put_u8      (p, uint8_t(_conf.temp_fet_end));   // l_temp_fet_end   ← param
    put_u8      (p, 85);                       // l_temp_motor_start
    put_u8      (p, 105);                      // l_temp_motor_end
    put_f16     (p, 0.15f, 10000.0f);          // l_temp_accel_dec
    put_f16     (p, 0.005f, 10000.0f);         // l_min_duty
    // Converted to VESC's modulation semantic — see duty_max_to_vesc().
    put_f16     (p, duty_max_to_vesc(_mc.get_duty_max()), 10000.0f); // l_max_duty ← param (D_MAX)
    put_f32_auto(p, 500000.0f);                // l_watt_max
    put_f32_auto(p, -500000.0f);               // l_watt_min
    put_f16     (p, 1.0f, 10000.0f);           // l_current_max_scale
    put_f16     (p, 1.0f, 10000.0f);           // l_current_min_scale
    put_f16     (p, _mc.get_duty_start(), 10000.0f); // l_duty_start    ← param (D_START)
    put_f32_auto(p, 150.0f);                   // sl_min_erpm
    put_f32_auto(p, 1100.0f);                  // sl_min_erpm_cycle_int_limit
    put_f32_auto(p, 10.0f);                    // sl_max_fullbreak_current_dir_change
    put_f16     (p, 62.0f, 10.0f);             // sl_cycle_int_limit
    put_f16     (p, 0.8f, 10000.0f);           // sl_phase_advance_at_br
    put_f32_auto(p, 80000.0f);                 // sl_cycle_int_rpm_br
    put_f32_auto(p, 600.0f);                   // sl_bemf_coupling_k
    for (uint8_t k = 0; k < 8; k++) put_u8(p, 255); // hall_table[0-7] (BLDC, unused)
    put_f32_auto(p, 2000.0f);                  // hall_sl_erpm
    put_f32_auto(p, kp);                       // foc_current_kp       ← live
    put_f32_auto(p, ki);                       // foc_current_ki       ← live
    put_f32_auto(p, float(_mc.get_pwm_rate_hz()));   // foc_f_zv       ← live PWM rate
    put_f32_auto(p, float(_mc.deadtime_ns()) * 1e-3f); // foc_dt_us    ← achieved dead time
    put_u8      (p, 0);                        // foc_encoder_inverted
    put_f32_auto(p, 0.0f);                     // foc_encoder_offset
    put_f32_auto(p, 7.0f);                     // foc_encoder_ratio
    put_u8      (p, is_hall ? 2 : 0);          // foc_sensor_mode (2 = HALL) ← param
    put_f32_auto(p, 2000.0f);                  // foc_pll_kp
    put_f32_auto(p, 30000.0f);                 // foc_pll_ki
    put_f32_auto(p, L);                        // foc_motor_l          ← param
    put_f32_auto(p, 0.0f);                     // foc_motor_ld_lq_diff
    put_f32_auto(p, R);                        // foc_motor_r          ← param
    put_f32_auto(p, flux);                     // foc_motor_flux_linkage ← param
    put_f32_auto(p, _conf.observer_gain);      // foc_observer_gain      ← param
    put_f32_auto(p, 0.05f);                    // foc_observer_gain_slow
    put_f16     (p, 0.0f, 1000.0f);            // foc_observer_offset
    put_f32_auto(p, 10.0f);                    // foc_duty_dowmramp_kp
    put_f32_auto(p, 200.0f);                   // foc_duty_dowmramp_ki
    put_f16     (p, 1.0f, 10000.0f);           // foc_start_curr_dec
    put_f32_auto(p, 2500.0f);                  // foc_start_curr_dec_rpm
    put_f32_auto(p, ol_erpm);                  // foc_openloop_rpm     ← param (OL_ERPM)
    put_f16     (p, ol_rpmlow, 1000.0f);       // foc_openloop_rpm_low ← param (OL_LOW)
    put_f16     (p, 1.0f, 1000.0f);            // foc_d_gain_scale_start
    put_f16     (p, 0.2f, 1000.0f);            // foc_d_gain_scale_max_mod
    put_f16     (p, ol_hyst, 100.0f);          // foc_sl_openloop_hyst      ← param (OL_HYST)
    put_f16     (p, ol_tlock, 100.0f);         // foc_sl_openloop_time_lock ← param (OL_TLOCK)
    put_f16     (p, ol_tramp, 100.0f);         // foc_sl_openloop_time_ramp ← param (OL_TRAMP)
    put_f16     (p, ol_tconst, 100.0f);        // foc_sl_openloop_time      ← param (OL_TCONST)
    put_f16     (p, ol_boost, 100.0f);         // foc_sl_openloop_boost_q   ← param (OL_IBOOST)
    put_f16     (p, ol_maxq, 100.0f);          // foc_sl_openloop_max_q     ← param (OL_IMAX)
    for (uint8_t k = 0; k < 8; k++) {          // foc_hall_table[0-7]  ← param
        if (isnan(hd[k])) {
            put_u8(p, 255);                    // unmapped state
        } else {
            float a = hd[k];
            while (a < 0.0f)    a += 360.0f;
            while (a >= 360.0f) a -= 360.0f;
            put_u8(p, uint8_t(lroundf(a * (200.0f / 360.0f)) % 200));
        }
    }
    put_f32_auto(p, _mc.get_hall_interp_erpm());// foc_hall_interp_erpm ← param
    put_f32_auto(p, bl_lo);                    // foc_sl_erpm_start    ← param (hall blend lo)
    put_f32_auto(p, bl_hi);                    // foc_sl_erpm          ← param (hall blend hi)
    put_u8      (p, 0);                        // foc_control_sample_mode
    put_u8      (p, 0);                        // foc_current_sample_mode
    put_u8      (p, 0);                        // foc_sat_comp_mode
    put_f16     (p, 0.0f, 1000.0f);            // foc_sat_comp
    put_u8      (p, 0);                        // foc_temp_comp
    put_f16     (p, 25.0f, 100.0f);            // foc_temp_comp_base_temp
    put_f16     (p, 0.1f, 10000.0f);           // foc_current_filter_const
    put_u8      (p, 0);                        // foc_cc_decoupling
    put_u8      (p, 0);                        // foc_observer_type
    put_u8      (p, 0);                        // foc_hfi_amb_mode
    put_f16     (p, 0.0f, 10.0f);              // foc_hfi_amb_current
    put_u8      (p, 0);                        // foc_hfi_amb_tres
    put_f16     (p, 5.0f, 10.0f);              // foc_hfi_voltage_start
    put_f16     (p, 2.0f, 10.0f);              // foc_hfi_voltage_run
    put_f16     (p, 10.0f, 10.0f);             // foc_hfi_voltage_max
    put_f16     (p, 1.0f, 1000.0f);            // foc_hfi_gain
    put_f16     (p, 1.0f, 1000.0f);            // foc_hfi_max_err
    put_f16     (p, 1.0f, 100.0f);             // foc_hfi_hyst
    put_f32_auto(p, 2000.0f);                  // foc_sl_erpm_hfi
    put_u16     (p, 65);                       // foc_hfi_start_samples
    put_f32_auto(p, 0.001f);                   // foc_hfi_obs_ovr_sec
    put_u8      (p, 0);                        // foc_hfi_samples
    put_u8      (p, 1);                        // foc_offsets_cal_mode
    for (uint8_t k = 0; k < 3; k++) put_f32_auto(p, 2048.0f);       // foc_offsets_current[0-2]
    for (uint8_t k = 0; k < 3; k++) put_f16(p, 0.0f, 10000.0f);     // foc_offsets_voltage[0-2]
    for (uint8_t k = 0; k < 3; k++) put_f16(p, 0.0f, 10000.0f);     // foc_offsets_voltage_undriven[0-2]
    put_u8      (p, 0);                        // foc_phase_filter_enable
    put_u8      (p, 0);                        // foc_phase_filter_disable_fault
    put_f32_auto(p, 4000.0f);                  // foc_phase_filter_max_erpm
    put_u8      (p, 0);                        // foc_mtpa_mode
    put_f32_auto(p, 0.0f);                     // foc_fw_current_max
    put_f16     (p, 0.9f, 10000.0f);           // foc_fw_duty_start
    put_f16     (p, 0.2f, 1000.0f);            // foc_fw_ramp_time
    put_f16     (p, 0.02f, 10000.0f);          // foc_fw_q_current_factor
    put_u8      (p, 0);                        // foc_speed_soure
    put_u8      (p, 0);                        // foc_short_ls_on_zero_duty
    put_f16     (p, 1.0f, 10000.0f);           // foc_overmod_factor
    put_u8      (p, 0);                        // sp_pid_loop_rate
    put_f32_auto(p, 0.004f);                   // s_pid_kp
    put_f32_auto(p, 0.004f);                   // s_pid_ki
    put_f32_auto(p, 0.0001f);                  // s_pid_kd
    put_f16     (p, 0.2f, 10000.0f);           // s_pid_kd_filter
    put_f32_auto(p, 900.0f);                   // s_pid_min_erpm
    put_u8      (p, 1);                        // s_pid_allow_braking
    put_f32_auto(p, 25000.0f);                 // s_pid_ramp_erpms_s
    put_u8      (p, 0);                        // s_pid_speed_source
    put_f32_auto(p, 0.03f);                    // p_pid_kp
    put_f32_auto(p, 0.0f);                     // p_pid_ki
    put_f32_auto(p, 0.0004f);                  // p_pid_kd
    put_f32_auto(p, 0.0004f);                  // p_pid_kd_proc
    put_f16     (p, 0.2f, 10000.0f);           // p_pid_kd_filter
    put_f32_auto(p, 1.0f);                     // p_pid_ang_div
    put_f16     (p, 0.0f, 10.0f);              // p_pid_gain_dec_angle
    put_f32_auto(p, 0.0f);                     // p_pid_offset
    put_f16     (p, 0.01f, 10000.0f);          // cc_startup_boost_duty
    put_f32_auto(p, 0.0f);                     // cc_min_current
    put_f32_auto(p, 0.0046f);                  // cc_gain
    put_f16     (p, 0.04f, 10000.0f);          // cc_ramp_step_max
    put_i32     (p, 500);                      // m_fault_stop_time_ms
    put_f16     (p, 0.02f, 10000.0f);          // m_duty_ramp_step
    put_f32_auto(p, 0.5f);                     // m_current_backoff_gain
    put_u32     (p, 8192);                     // m_encoder_counts
    put_f16     (p, 0.0f, 1000.0f);            // m_encoder_sin_amp
    put_f16     (p, 0.0f, 1000.0f);            // m_encoder_cos_amp
    put_f16     (p, 0.0f, 1000.0f);            // m_encoder_sin_offset
    put_f16     (p, 0.0f, 1000.0f);            // m_encoder_cos_offset
    put_f16     (p, 0.5f, 1000.0f);            // m_encoder_sincos_filter_constant
    put_f16     (p, 0.0f, 1000.0f);            // m_encoder_sincos_phase_correction
    put_u8      (p, 0);                        // m_sensor_port_mode
    put_u8      (p, 0);                        // m_invert_direction
    put_u8      (p, 0);                        // m_drv8301_oc_mode
    put_u8      (p, 16);                       // m_drv8301_oc_adj
    put_f32_auto(p, 3000.0f);                  // m_bldc_f_sw_min
    put_f32_auto(p, 30000.0f);                 // m_bldc_f_sw_max
    put_f32_auto(p, 25000.0f);                 // m_dc_f_sw
    put_f32_auto(p, 3380.0f);                  // m_ntc_motor_beta
    put_u8      (p, 0);                        // m_out_aux_mode
    put_u8      (p, 0);                        // m_motor_temp_sens_type
    put_f32_auto(p, 0.61f);                    // m_ptc_motor_coeff
    put_f16     (p, 10000.0f, 0.1f);           // m_ntcx_ptcx_res
    put_f16     (p, 25.0f, 10.0f);             // m_ntcx_ptcx_temp_base
    put_u8      (p, 0);                        // m_hall_extra_samples
    put_u8      (p, 10);                       // m_batt_filter_const
    put_u8      (p, _conf.poles);              // si_motor_poles       ← param (pole count)
    put_f32_auto(p, 1.0f);                     // si_gear_ratio
    put_f32_auto(p, 0.083f);                   // si_wheel_diameter
    put_u8      (p, 0);                        // si_battery_type
    put_u8      (p, 3);                        // si_battery_cells
    put_f32_auto(p, 0.0f);                     // si_battery_ah
    put_f32_auto(p, 0.0f);                     // si_motor_nl_current
    put_u8      (p, 0);                        // bms.type
    put_u8      (p, 0);                        // bms.limit_mode
    put_u8      (p, 0);                        // bms.t_limit_start
    put_u8      (p, 0);                        // bms.t_limit_end
    put_f16     (p, 0.0f, 1000.0f);            // bms.soc_limit_start
    put_f16     (p, 0.0f, 1000.0f);            // bms.soc_limit_end
    put_f16     (p, 0.0f, 1000.0f);            // bms.vmin_limit_start
    put_f16     (p, 0.0f, 1000.0f);            // bms.vmin_limit_end
    put_f16     (p, 0.0f, 1000.0f);            // bms.vmax_limit_start
    put_f16     (p, 0.0f, 1000.0f);            // bms.vmax_limit_end
    put_u8      (p, 0);                        // bms.fwd_can_mode

    send_packet(buf, uint16_t(p - buf));
}

// Parse VESC Tool's "Write Motor Configuration" (COMM_SET_MCCONF) blob in the
// 6.06 field order and pull out the standard fields we back with params. Every
// field must be consumed in order (fixed widths) so the ones we want land at the
// right offset; fields we don't map are skipped by advancing the cursor. The
// custom protections (stall/regen/vbus-fold/slew/current-scale) have no VESC
// mc_configuration slot and are untouched here — set those via DroneCAN params.
void VescTelemetry::handle_set_mcconf()
{
    // Blob starts after the command-id byte; first word is the signature.
    const uint8_t *p = &_payload[1];
    if (_payload_len < 1 + 4 || get_u32(p) != MCCONF_SIGNATURE) {
        return;   // wrong FW/version layout — ignore (VESC Tool shows an error)
    }

    // Skip helpers: consume one field of the given type, advancing the cursor.
    auto A  = [&]()          { (void)get_f32_auto(p); };
    auto H  = [&](float s)   { (void)get_f16(p, s); };
    auto U8 = [&]()          { p += 1; };
    auto U16= [&]()          { p += 2; };
    auto U32= [&]()          { p += 4; };
    auto I32= [&]()          { p += 4; };

    McconfIn in{};

    U8(); U8(); U8(); U8();                 // pwm_mode, comm_mode, motor_type, sensor_mode
    in.current_max = get_f32_auto(p);       // l_current_max
    in.current_min = get_f32_auto(p);       // l_current_min (negative = braking cap)
    A(); A();                               // l_in_current_max, l_in_current_min
    H(10000); H(10000);                     // l_in_current_map_start, _filter
    in.abs_current_max = get_f32_auto(p);   // l_abs_current_max
    in.min_erpm   = get_f32_auto(p);        // l_min_erpm
    in.max_erpm   = get_f32_auto(p);        // l_max_erpm
    in.erpm_start = get_f16(p, 10000);      // l_erpm_start
    A(); A();                               // l_max_erpm_fbrake, _cc
    H(10);                                  // l_min_vin
    in.max_vin = get_f16(p, 10);            // l_max_vin
    H(10); H(10);                           // l_battery_cut_start, _end
    in.regen_cut_start = get_f16(p, 10);    // l_battery_regen_cut_start
    in.regen_cut_end   = get_f16(p, 10);    // l_battery_regen_cut_end
    U8();                                   // l_slow_abs_current
    in.temp_fet_start = float(*p); p += 1;  // l_temp_fet_start (u8)
    in.temp_fet_end   = float(*p); p += 1;  // l_temp_fet_end (u8)
    U8(); U8();                             // l_temp_motor_start, _end
    H(10000); H(10000);                     // l_temp_accel_dec, l_min_duty
    // l_max_duty arrives as a MODULATION ceiling; store the equivalent per-phase
    // D_MAX so the sink's range check and the param are in the same units.
    in.max_duty = vesc_to_duty_max(get_f16(p, 10000));  // l_max_duty
    A(); A();                               // l_watt_max, l_watt_min
    H(10000); H(10000);                     // l_current_max_scale, l_current_min_scale
    in.duty_start = get_f16(p, 10000);      // l_duty_start
    A(); A(); A();                          // sl_min_erpm, sl_min_erpm_cycle_int_limit, sl_max_fullbreak_..
    H(10);                                  // sl_cycle_int_limit
    H(10000);                               // sl_phase_advance_at_br
    A(); A();                               // sl_cycle_int_rpm_br, sl_bemf_coupling_k
    p += 8;                                 // hall_table[0-7] u8
    A();                                    // hall_sl_erpm
    in.current_kp = get_f32_auto(p);        // foc_current_kp
    in.current_ki = get_f32_auto(p);        // foc_current_ki
    A(); A();                               // foc_f_zv, foc_dt_us
    U8();                                   // foc_encoder_inverted
    A(); A();                               // foc_encoder_offset, foc_encoder_ratio
    in.sensor_mode = p[0]; p += 1;          // foc_sensor_mode (0=sensorless, 2=hall)
    A(); A();                               // foc_pll_kp, foc_pll_ki
    in.motor_l = get_f32_auto(p);           // foc_motor_l
    A();                                    // foc_motor_ld_lq_diff
    in.motor_r    = get_f32_auto(p);        // foc_motor_r
    in.motor_flux = get_f32_auto(p);        // foc_motor_flux_linkage
    in.observer_gain = get_f32_auto(p);      // foc_observer_gain
    A();                                    // foc_observer_gain_slow
    H(1000);                                // foc_observer_offset
    A(); A();                               // foc_duty_dowmramp_kp, _ki
    H(10000);                               // foc_start_curr_dec
    A();                                    // foc_start_curr_dec_rpm
    in.ol_erpm    = get_f32_auto(p);        // foc_openloop_rpm
    in.ol_rpm_low = get_f16(p, 1000);       // foc_openloop_rpm_low
    H(1000); H(1000);                       // foc_d_gain_scale_start, _max_mod
    in.ol_hyst    = get_f16(p, 100);        // foc_sl_openloop_hyst
    in.ol_t_lock  = get_f16(p, 100);        // foc_sl_openloop_time_lock
    in.ol_t_ramp  = get_f16(p, 100);        // foc_sl_openloop_time_ramp
    in.ol_t_const = get_f16(p, 100);        // foc_sl_openloop_time
    in.ol_boost_q = get_f16(p, 100);        // foc_sl_openloop_boost_q
    in.ol_max_q   = get_f16(p, 100);        // foc_sl_openloop_max_q
    p += 8;                                 // foc_hall_table[0-7] u8
    in.hall_interp_erpm = get_f32_auto(p);  // foc_hall_interp_erpm
    in.sl_erpm_start    = get_f32_auto(p);  // foc_sl_erpm_start
    in.sl_erpm          = get_f32_auto(p);  // foc_sl_erpm
    U8(); U8(); U8();                       // foc_control_sample_mode, foc_current_sample_mode, foc_sat_comp_mode
    H(1000);                                // foc_sat_comp
    U8();                                   // foc_temp_comp
    H(100);                                 // foc_temp_comp_base_temp
    H(10000);                               // foc_current_filter_const
    U8(); U8();                             // foc_cc_decoupling, foc_observer_type
    U8();                                   // foc_hfi_amb_mode
    H(10);                                  // foc_hfi_amb_current
    U8();                                   // foc_hfi_amb_tres
    H(10); H(10); H(10);                    // foc_hfi_voltage_start, _run, _max
    H(1000); H(1000);                       // foc_hfi_gain, foc_hfi_max_err
    H(100);                                 // foc_hfi_hyst
    A();                                    // foc_sl_erpm_hfi
    U16();                                  // foc_hfi_start_samples
    A();                                    // foc_hfi_obs_ovr_sec
    U8(); U8();                             // foc_hfi_samples, foc_offsets_cal_mode
    A(); A(); A();                          // foc_offsets_current[0-2]
    H(10000); H(10000); H(10000);           // foc_offsets_voltage[0-2]
    H(10000); H(10000); H(10000);           // foc_offsets_voltage_undriven[0-2]
    U8(); U8();                             // foc_phase_filter_enable, _disable_fault
    A();                                    // foc_phase_filter_max_erpm
    U8();                                   // foc_mtpa_mode
    A();                                    // foc_fw_current_max
    H(10000); H(1000); H(10000);            // foc_fw_duty_start, _ramp_time, _q_current_factor
    U8(); U8();                             // foc_speed_soure, foc_short_ls_on_zero_duty
    H(10000);                               // foc_overmod_factor
    U8();                                   // sp_pid_loop_rate
    A(); A(); A();                          // s_pid_kp, ki, kd
    H(10000);                               // s_pid_kd_filter
    A();                                    // s_pid_min_erpm
    U8();                                   // s_pid_allow_braking
    A();                                    // s_pid_ramp_erpms_s
    U8();                                   // s_pid_speed_source
    A(); A(); A(); A();                     // p_pid_kp, ki, kd, kd_proc
    H(10000);                               // p_pid_kd_filter
    A();                                    // p_pid_ang_div
    H(10);                                  // p_pid_gain_dec_angle
    A();                                    // p_pid_offset
    H(10000);                               // cc_startup_boost_duty
    A(); A();                               // cc_min_current, cc_gain
    H(10000);                               // cc_ramp_step_max
    I32();                                  // m_fault_stop_time_ms
    H(10000);                               // m_duty_ramp_step
    A();                                    // m_current_backoff_gain
    U32();                                  // m_encoder_counts
    H(1000); H(1000); H(1000); H(1000); H(1000); H(1000); // encoder sin/cos amp/offset, sincos filter/phase
    U8(); U8(); U8(); U8();                 // m_sensor_port_mode, m_invert_direction, m_drv8301_oc_mode, _oc_adj
    A(); A(); A(); A();                     // m_bldc_f_sw_min, _max, m_dc_f_sw, m_ntc_motor_beta
    U8(); U8();                             // m_out_aux_mode, m_motor_temp_sens_type
    A();                                    // m_ptc_motor_coeff
    H(0.1f); H(10);                         // m_ntcx_ptcx_res, m_ntcx_ptcx_temp_base
    U8(); U8();                             // m_hall_extra_samples, m_batt_filter_const
    in.poles = *p; p += 1;                  // si_motor_poles (u8) — last field we need

    // Guard against a short/truncated frame walking past the payload.
    if (uintptr_t(p - &_payload[0]) > _payload_len) {
        return;
    }

    if (_conf_cb != nullptr) {
        _conf_cb(_conf_ctx, in);   // persist + schedule reboot to apply
    }
    // Ack with the bare command id (VESC commands.c convention) so VESC Tool
    // reports the write succeeded before we reboot.
    uint8_t ack = COMM_SET_MCCONF;
    send_packet(&ack, 1);
}

// Serialize a full VESC FW 6.06 app_configuration. VESC Tool will not open its
// App Settings pages at all without a complete, signature-valid blob, so every
// one of the 119 fields is emitted in order even though this firmware implements
// almost none of them — the rest are fixed stand-ins chosen to look like a stock
// VESC so the pages render sanely rather than showing zeros everywhere.
//
// Only the PPM block means anything here. app_to_use reports APP_PPM (1) so the
// tool opens on the page whose Control Type this ESC actually obeys.
void VescTelemetry::handle_get_appconf(uint8_t reply_id)
{
    uint8_t buf[512];
    uint8_t *p = buf;

    put_u8      (p, reply_id);                 // packet id (echoes request)
    put_u32     (p, APPCONF_SIGNATURE);

    put_u8      (p, 0);                        // controller_id
    put_u32     (p, 1000);                     // timeout_msec
    put_f32_auto(p, 0.0f);                     // timeout_brake_current
    put_u16     (p, 50);                       // can_status_rate_1
    put_u16     (p, 5);                        // can_status_rate_2
    put_u8      (p, 0);                        // can_status_msgs_r1
    put_u8      (p, 0);                        // can_status_msgs_r2
    put_u8      (p, 2);                        // can_baud_rate = 500K
    put_u8      (p, 0);                        // pairing_done
    put_u8      (p, 1);                        // permanent_uart_enabled
    put_u8      (p, 0);                        // shutdown_mode
    put_u8      (p, 0);                        // can_mode = CAN_MODE_VESC
    put_u8      (p, 0);                        // uavcan_esc_index
    put_u8      (p, 0);                        // uavcan_raw_mode
    put_f32_auto(p, 50000.0f);                 // uavcan_raw_rpm_max
    put_u8      (p, 0);                        // uavcan_status_current_mode
    put_u8      (p, 0);                        // servo_out_enable
    put_u8      (p, 0);                        // kill_sw_mode
    put_u8      (p, 1);                        // app_to_use = APP_PPM

    // ── PPM. The two live fields, plus honest values for the pulse geometry the
    // decode actually uses so the page does not misdescribe the hardware. Those
    // three are still compile-time constants here (THR_PWM_* in foc_esc.cpp);
    // they are reported, not accepted back.
    put_u8      (p, _conf.ppm_ctrl_type);      // ctrl_type       ← param (PPM_CTRL)
    put_f32_auto(p, 15000.0f);                 // pid_max_erpm
    put_f32_auto(p, 0.08f);                    // hyst  = 40 us / 500 us
    put_f32_auto(p, 1.0f);                     // pulse_start  [ms]
    put_f32_auto(p, 2.0f);                     // pulse_end    [ms]
    put_f32_auto(p, 1.5f);                     // pulse_center [ms]
    put_u8      (p, 0);                        // median_filter
    put_u8      (p, 1);                        // safe_start = REGULAR (we do arm through neutral)
    put_f32_auto(p, 0.0f);                     // throttle_exp
    put_f32_auto(p, 0.0f);                     // throttle_exp_brake
    put_u8      (p, 2);                        // throttle_exp_mode = THR_EXP_POLY
    put_f32_auto(p, 0.0f);                     // ramp_time_pos — no PPM-side ramp
    put_f32_auto(p, 0.0f);                     // ramp_time_neg   (see I_SLEW instead)
    put_u8      (p, 0);                        // multi_esc
    put_u8      (p, 0);                        // tc
    put_f32_auto(p, 3000.0f);                  // tc_max_diff
    put_f16     (p, _conf.max_erpm_for_dir, 1.0f); // max_erpm_for_dir ← param (DIR_ERPM)
    put_f32_auto(p, 0.07f);                    // smart_rev_max_duty
    put_f32_auto(p, 3.0f);                     // smart_rev_ramp_time

    // ── ADC app: not implemented. Stock-looking values, ctrl_type = off.
    put_u8      (p, 0);                        // adc ctrl_type
    put_f32_auto(p, 0.15f);                    // adc hyst
    put_f16     (p, 0.8f,  1000.0f);           // voltage_start
    put_f16     (p, 3.1f,  1000.0f);           // voltage_end
    put_f16     (p, 0.0f,  1000.0f);           // voltage_min
    put_f16     (p, 3.3f,  1000.0f);           // voltage_max
    put_f16     (p, 1.55f, 1000.0f);           // voltage_center
    put_f16     (p, 0.8f,  1000.0f);           // voltage2_start
    put_f16     (p, 3.1f,  1000.0f);           // voltage2_end
    put_u8      (p, 1);                        // use_filter
    put_u8      (p, 1);                        // safe_start
    put_u8      (p, 0);                        // buttons
    put_u8      (p, 0);                        // voltage_inverted
    put_u8      (p, 0);                        // voltage2_inverted
    put_f32_auto(p, 0.0f);                     // throttle_exp
    put_f32_auto(p, 0.0f);                     // throttle_exp_brake
    put_u8      (p, 2);                        // throttle_exp_mode
    put_f32_auto(p, 0.4f);                     // ramp_time_pos
    put_f32_auto(p, 0.2f);                     // ramp_time_neg
    put_u8      (p, 0);                        // multi_esc
    put_u8      (p, 0);                        // tc
    put_f32_auto(p, 3000.0f);                  // tc_max_diff
    put_u16     (p, 500);                      // update_rate_hz

    put_u32     (p, 115200);                   // app_uart_baudrate

    // ── Nunchuk app: not implemented.
    put_u8      (p, 0);                        // chuk ctrl_type
    put_f32_auto(p, 0.15f);                    // hyst
    put_f32_auto(p, 0.4f);                     // ramp_time_pos
    put_f32_auto(p, 0.2f);                     // ramp_time_neg
    put_f32_auto(p, 3000.0f);                  // stick_erpm_per_s_in_cc
    put_f32_auto(p, 0.0f);                     // throttle_exp
    put_f32_auto(p, 0.0f);                     // throttle_exp_brake
    put_u8      (p, 2);                        // throttle_exp_mode
    put_u8      (p, 0);                        // multi_esc
    put_u8      (p, 0);                        // tc
    put_f32_auto(p, 3000.0f);                  // tc_max_diff
    put_u8      (p, 0);                        // use_smart_rev
    put_f32_auto(p, 0.07f);                    // smart_rev_max_duty
    put_f32_auto(p, 3.0f);                     // smart_rev_ramp_time

    // ── NRF pairing: no radio on this board.
    put_u8(p, 0); put_u8(p, 0); put_u8(p, 0); put_u8(p, 0);  // speed, power, crc_type, retry_delay
    put_u8(p, 0); put_u8(p, 0);                              // retries, channel
    put_u8(p, 0); put_u8(p, 0); put_u8(p, 0);                // address[0..2]
    put_u8(p, 0);                                            // send_crc_ack

    // ── PAS (pedal assist): not implemented.
    put_u8      (p, 0);                        // pas ctrl_type
    put_u8      (p, 0);                        // sensor_type
    put_f16     (p, 0.5f, 1000.0f);            // current_scaling
    put_f16     (p, 10.0f, 10.0f);             // pedal_rpm_start
    put_f16     (p, 50.0f, 10.0f);             // pedal_rpm_end
    put_u8      (p, 0);                        // invert_pedal_direction
    put_u16     (p, 24);                       // magnets
    put_u8      (p, 1);                        // use_filter
    put_f16     (p, 0.4f, 100.0f);             // ramp_time_pos
    put_f16     (p, 0.2f, 100.0f);             // ramp_time_neg
    put_u16     (p, 500);                      // update_rate_hz

    // ── IMU: type = OFF. AP owns any IMU on this vehicle, not the ESC.
    put_u8      (p, 0);                        // imu type = IMU_TYPE_OFF
    put_u8      (p, 0);                        // mode
    put_u8      (p, 0);                        // filter
    put_f16     (p, 15.0f, 1.0f);              // accel_lowpass_filter_x
    put_f16     (p, 15.0f, 1.0f);              // accel_lowpass_filter_y
    put_f16     (p, 15.0f, 1.0f);              // accel_lowpass_filter_z
    put_f16     (p, 15.0f, 1.0f);              // gyro_lowpass_filter
    put_u16     (p, 200);                      // sample_rate_hz
    put_u8      (p, 0);                        // use_magnetometer
    put_f32_auto(p, 1.0f);                     // accel_confidence_decay
    put_f32_auto(p, 0.3f);                     // mahony_kp
    put_f32_auto(p, 0.0f);                     // mahony_ki
    put_f32_auto(p, 0.1f);                     // madgwick_beta
    put_f32_auto(p, 0.0f);                     // rot_roll
    put_f32_auto(p, 0.0f);                     // rot_pitch
    put_f32_auto(p, 0.0f);                     // rot_yaw
    put_f32_auto(p, 0.0f); put_f32_auto(p, 0.0f); put_f32_auto(p, 0.0f); // accel_offsets[0..2]
    put_f32_auto(p, 0.0f); put_f32_auto(p, 0.0f); put_f32_auto(p, 0.0f); // gyro_offsets[0..2]

    send_packet(buf, uint16_t(p - buf));
}

// Parse VESC Tool's "Write App Configuration" (COMM_SET_APPCONF). Same shape as
// handle_set_mcconf: walk the 6.06 field order with typed skips so the cursor
// stays aligned, and lift out only what the PWM decode consumes. The skips are
// not decoration — a field consumed at the wrong width silently shifts every
// field after it, and the two we want sit 30 and 75 bytes into the blob.
void VescTelemetry::handle_set_appconf()
{
    _appconf_rx++;
    const uint8_t *p = &_payload[1];
    if (_payload_len < 1 + 4) {
        _appconf_short++;
        return;
    }
    if (get_u32(p) != APPCONF_SIGNATURE) {
        _appconf_badsig++;
        return;   // wrong FW/version layout — ignore (VESC Tool shows an error)
    }

    // No f16 skip helper here: nothing before max_erpm_for_dir is a float16.
    auto A  = [&]()          { (void)get_f32_auto(p); };
    auto U8 = [&]()          { p += 1; };
    auto U16= [&]()          { p += 2; };
    auto U32= [&]()          { p += 4; };

    AppconfIn in{};

    U8();                                   // controller_id
    U32();                                  // timeout_msec
    A();                                    // timeout_brake_current
    U16(); U16();                           // can_status_rate_1, _2
    U8(); U8(); U8();                       // can_status_msgs_r1, _r2, can_baud_rate
    U8(); U8(); U8(); U8();                 // pairing_done, permanent_uart_enabled, shutdown_mode, can_mode
    U8(); U8();                             // uavcan_esc_index, uavcan_raw_mode
    A();                                    // uavcan_raw_rpm_max
    U8(); U8(); U8();                       // uavcan_status_current_mode, servo_out_enable, kill_sw_mode
    U8();                                   // app_to_use
    in.ppm_ctrl_type = *p; p += 1;          // app_ppm_conf.ctrl_type
    A(); A(); A(); A(); A();                // pid_max_erpm, hyst, pulse_start, _end, _center
    U8(); U8();                             // median_filter, safe_start
    A(); A();                               // throttle_exp, throttle_exp_brake
    U8();                                   // throttle_exp_mode
    A(); A();                               // ramp_time_pos, _neg
    U8(); U8();                             // multi_esc, tc
    A();                                    // tc_max_diff
    in.max_erpm_for_dir = get_f16(p, 1);    // max_erpm_for_dir — last field we need

    // Guard against a short/truncated frame walking past the payload.
    if (uintptr_t(p - &_payload[0]) > _payload_len) {
        _appconf_short++;
        return;
    }

    _appconf_ctrl = in.ppm_ctrl_type;
    _appconf_dir  = in.max_erpm_for_dir;

    if (_appconf_cb != nullptr) {
        _appconf_cb(_conf_ctx, in);   // persist + schedule reboot to apply
    }
    uint8_t ack = COMM_SET_APPCONF;
    send_packet(&ack, 1);
}

} // namespace ChibiOS

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
