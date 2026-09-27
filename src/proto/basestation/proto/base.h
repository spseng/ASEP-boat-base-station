#pragma once

// Messages local to the PC <-> base-station ESP32 serial link.
//
// The base-station ESP32 relays wirelink frames between USB serial and LoRa
// unchanged. In addition it may emit the messages below, which are never
// transmitted over the air. Their type numbers start at 0x80 so they can
// never collide with a LoRa MsgType. All are optional: the app works
// without them and simply shows nothing in the corresponding panels.

#include <cstdint>

namespace basestation::base {

    constexpr uint8_t FIRST_LOCAL_TYPE = 0x80;

    enum class MsgType : uint8_t {
        RxInfo     = 0x80,
        BaseStatus = 0x81,
        BasePosition = 0x82,
    };

    constexpr bool is_local_type(uint8_t type) { return type >= FIRST_LOCAL_TYPE; }

    // Sent immediately after each relayed LoRa frame; describes how well the
    // base station received that frame. The app attaches it to the frame
    // that precedes it on the serial link.
    struct RxInfo {
        constexpr static MsgType TYPE = MsgType::RxInfo;
        float rssi;  // dBm
        float snr;   // dB

        template <class F>
        void fields(F& f) { f(rssi); f(snr); }
    };

    // Base-station ESP32 health, ~1 Hz.
    struct BaseStatus {
        constexpr static MsgType TYPE = MsgType::BaseStatus;
        uint32_t uptime_ms;  // resets reveal an ESP32 reboot
        uint32_t rx_ok;      // LoRa packets received and relayed
        uint32_t rx_bad;     // LoRa packets dropped (CRC error / too long)
        uint32_t tx_count;   // frames transmitted over LoRa

        template <class F>
        void fields(F& f) { f(uptime_ms); f(rx_ok); f(rx_bad); f(tx_count); }
    };

    // Base-station position, ~1 Hz, only if the base ESP32 has a GPS. When
    // it reports a fix it overrides the position the operator set by hand
    // (unless they pin the manual one). lat/lon are meaningless without a fix.
    struct BasePosition {
        constexpr static MsgType TYPE = MsgType::BasePosition;
        int32_t lat;          // 1e-7 deg
        int32_t lon;          // 1e-7 deg
        uint8_t fix_quality;  // 0 = no fix (NMEA GGA quality otherwise)
        uint8_t satellites;

        template <class F>
        void fields(F& f) { f(lat); f(lon); f(fix_quality); f(satellites); }
    };
}  // namespace basestation::base
