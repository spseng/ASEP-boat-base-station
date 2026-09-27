// asep_log_dump: prints a recorded .aseplog session as CSV.
//
//   unix_us,dir,type,type_name,seq,len,fields
//
// `fields` is a space-separated key=value list decoded with
// basestation::codec (best effort), or the payload as hex with --raw.
// Values never contain commas, so no CSV quoting is needed.

#include <basestation/core/frame_log.h>
#include <basestation/proto/base.h>
#include <basestation/proto/codec.h>
#include <basestation/proto/lora.h>
#include <basestation/proto/names.h>

#include <boat_defs/ids.h>
#include <boat_defs/mode.h>
#include <boat_defs/units.h>

#include <cinttypes>
#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

namespace {

namespace lora = basestation::lora;
namespace base = basestation::base;
namespace codec = basestation::codec;
namespace names = basestation::names;

void usage(std::FILE* out) {
    std::fprintf(out,
        "usage: asep_log_dump [--raw] [--no-header] FILE.aseplog\n"
        "\n"
        "Prints every recorded frame as CSV on stdout:\n"
        "  unix_us,dir,type,type_name,seq,len,fields\n"
        "fields is 'key=value ...' decoded from the payload (best effort).\n"
        "\n"
        "  --raw        print the payload as hex instead of decoded fields\n"
        "  --no-header  omit the CSV header line\n"
        "  --help       show this help\n");
}

const char* type_name(uint8_t t) {
    switch (t) {
        case static_cast<uint8_t>(lora::MsgType::Disable): return "Disable";
        case static_cast<uint8_t>(lora::MsgType::SelfStatus): return "SelfStatus";
        case static_cast<uint8_t>(lora::MsgType::Command): return "Command";
        case static_cast<uint8_t>(lora::MsgType::Status): return "Status";
        case static_cast<uint8_t>(lora::MsgType::Reenable): return "Reenable";
        case static_cast<uint8_t>(lora::MsgType::SetMode): return "SetMode";
        case static_cast<uint8_t>(base::MsgType::RxInfo): return "RxInfo";
        case static_cast<uint8_t>(base::MsgType::BaseStatus): return "BaseStatus";
        case static_cast<uint8_t>(base::MsgType::BasePosition): return "BasePosition";
        default: return "unknown";
    }
}

std::string id_str(uint8_t id) {
    return id == boat::ids::BROADCAST_ID ? "ALL" : std::to_string(id);
}

// printf into a std::string (fields are short).
#if defined(__GNUC__)
__attribute__((format(printf, 1, 2)))
#endif
std::string fmt(const char* f, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    return buf;
}

std::string hex(const wirelink::Frame& f) {
    std::string s;
    for (size_t i = 0; i < f.len; ++i) s += fmt("%02x", f.payload.data[i]);
    return s;
}

std::string decode(const wirelink::Frame& f) {
    switch (f.type) {
        case static_cast<uint8_t>(lora::MsgType::Disable): {
            lora::Disable m{};
            if (codec::unpack(f, m)) return "rx_id=" + id_str(m.rx_id);
            break;
        }
        case static_cast<uint8_t>(lora::MsgType::Reenable): {
            lora::Reenable m{};
            if (codec::unpack(f, m)) return "rx_id=" + id_str(m.rx_id);
            break;
        }
        case static_cast<uint8_t>(lora::MsgType::SelfStatus): {
            lora::SelfStatus m{};
            if (codec::unpack(f, m)) {
                return fmt("id=%u lat=%.7f lon=%.7f scalar=%g age_ms=%" PRIu32,
                           unsigned(m.self.id), boat::units::e7_to_deg(m.self.lat),
                           boat::units::e7_to_deg(m.self.lon), double(m.self.scalar), m.self.age_ms);
            }
            break;
        }
        case static_cast<uint8_t>(lora::MsgType::Command): {
            lora::Command m{};
            if (codec::unpack(f, m)) {
                return "rx_id=" + id_str(m.rx_id) +
                       fmt(" lin_mm_s=%d ang_mrad_s=%d", int(m.lin_vel), int(m.ang_vel));
            }
            break;
        }
        case static_cast<uint8_t>(lora::MsgType::Status): {
            lora::Status m{};
            if (codec::unpack(f, m)) {
                return fmt("tx_id=%u mode=%s armed=%s gate=%s faults=0x%04x heading=%.1f gs_rssi=%.1f gs_snr=%.1f",
                           unsigned(m.tx_id), names::mode_name(m.mode).c_str(),
                           names::armed_name(m.armed).c_str(), names::gate_name(m.gate_state).c_str(), unsigned(m.fault_flags), double(m.heading_deg),
                           double(m.gs_rssi), double(m.gs_snr));
            }
            break;
        }
        case static_cast<uint8_t>(lora::MsgType::SetMode): {
            lora::SetMode m{};
            if (codec::unpack(f, m)) {
                return "rx_id=" + id_str(m.rx_id) +
                       fmt(" mode=%s armed=%s", names::mode_name(m.mode).c_str(),
                           names::armed_name(m.armed).c_str());
            }
            break;
        }
        case static_cast<uint8_t>(base::MsgType::RxInfo): {
            base::RxInfo m{};
            if (codec::unpack(f, m)) return fmt("rssi=%.1f snr=%.1f", double(m.rssi), double(m.snr));
            break;
        }
        case static_cast<uint8_t>(base::MsgType::BaseStatus): {
            base::BaseStatus m{};
            if (codec::unpack(f, m)) {
                return fmt("uptime_ms=%" PRIu32 " rx_ok=%" PRIu32 " rx_bad=%" PRIu32 " tx_count=%" PRIu32,
                           m.uptime_ms, m.rx_ok, m.rx_bad, m.tx_count);
            }
            break;
        }
        case static_cast<uint8_t>(base::MsgType::BasePosition): {
            base::BasePosition m{};
            if (codec::unpack(f, m)) {
                return fmt("lat=%.7f lon=%.7f fix_quality=%u satellites=%u", boat::units::e7_to_deg(m.lat),
                           boat::units::e7_to_deg(m.lon), unsigned(m.fix_quality), unsigned(m.satellites));
            }
            break;
        }
        default:
            return "raw=" + hex(f);
    }
    return "decode_error raw=" + hex(f);
}

}  // namespace

int main(int argc, char** argv) {
    bool raw = false, header = true;
    const char* path = nullptr;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--help") || !std::strcmp(argv[i], "-h")) {
            usage(stdout);
            return 0;
        } else if (!std::strcmp(argv[i], "--raw")) {
            raw = true;
        } else if (!std::strcmp(argv[i], "--no-header")) {
            header = false;
        } else if (argv[i][0] == '-' || path) {
            usage(stderr);
            return 2;
        } else {
            path = argv[i];
        }
    }
    if (!path) {
        usage(stderr);
        return 2;
    }

    basestation::FrameLogReader reader;
    std::string error;
    if (!reader.open(path, &error)) {
        std::fprintf(stderr, "asep_log_dump: %s: %s\n", path, error.c_str());
        return 1;
    }

    if (header) std::printf("unix_us,dir,type,type_name,seq,len,%s\n", raw ? "payload_hex" : "fields");
    basestation::LogRecord rec;
    while (reader.next(rec)) {
        const wirelink::Frame& f = rec.frame;
        std::printf("%" PRId64 ",%s,%u,%s,%u,%u,%s\n", rec.unix_us,
                    rec.dir == basestation::Direction::Tx ? "tx" : "rx",
                    unsigned(f.type), type_name(f.type), unsigned(f.seq), unsigned(f.len),
                    (raw ? hex(f) : decode(f)).c_str());
    }
    return 0;
}
