#include <basestation/core/msg_catalog.h>

#include <basestation/proto/base.h>
#include <basestation/proto/lora.h>
#include <basestation/proto/names.h>

#include <boat_defs/esc.h>
#include <boat_defs/ids.h>
#include <boat_defs/mode.h>

#include <wirelink/msg/serial.h>

#include <algorithm>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <type_traits>

namespace basestation::catalog {

namespace {

namespace lora = basestation::lora;
namespace base = basestation::base;
namespace ser = wirelink::msg::serial;
namespace ids = boat::ids;
using Problems = std::vector<Problem>;

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

void error(Problems& p, std::string text) { p.push_back({Severity::Error, std::move(text)}); }
void warn(Problems& p, std::string text) { p.push_back({Severity::Warning, std::move(text)}); }

// ---------------------------------------------------------------------------
// Field spec helpers
// ---------------------------------------------------------------------------

FieldSpec num(const char* name, const char* unit = "", int decimals = 2) {
    FieldSpec s{name};
    s.unit = unit;
    s.decimals = decimals;
    return s;
}
FieldSpec of(const char* name, Kind kind) {
    FieldSpec s{name};
    s.kind = kind;
    return s;
}
FieldSpec in(FieldSpec s, double lo, double hi) {
    s.lo = lo;
    s.hi = hi;
    return s;
}
FieldSpec nan_ok(FieldSpec s) {
    s.nan_ok = true;
    return s;
}
FieldSpec lat() { return in(of("lat", Kind::Lat), -90e7, 90e7); }
FieldSpec lon() { return in(of("lon", Kind::Lon), -180e7, 180e7); }
FieldSpec rssi(const char* name) { return in(num(name, "dBm", 1), -150, 0); }
FieldSpec snr(const char* name) { return in(num(name, "dB", 1), -30, 30); }

// PeerEntry and LinkStatus leaves, shared by several messages.
void peer_entry(std::vector<FieldSpec>& v) {
    v.insert(v.end(), {of("id", Kind::BoatId), lat(), lon(), num("scalar"), num("age", "ms")});
}
void link_status(std::vector<FieldSpec>& v) {
    v.insert(v.end(), {of("id", Kind::PeerId), rssi("rssi"), snr("snr"), num("lost")});
}
std::vector<FieldSpec> with_list(std::vector<FieldSpec> head, const char* list,
                                 void (*elem)(std::vector<FieldSpec>&)) {
    head.push_back(of(list, Kind::List));
    elem(head);
    return head;
}

// ---------------------------------------------------------------------------
// Flattener: walks a decoded message with its fields() visitor, pairing each
// leaf with the next FieldSpec to format and check it.
// ---------------------------------------------------------------------------

// The id/enum kinds are uint8 fields; clamp so other kinds never convert
// an out-of-range double (undefined behaviour).
uint8_t u8(double v) { return v >= 0 && v <= 255 ? static_cast<uint8_t>(v) : 0; }

std::string bound(double v) {
    return v == std::floor(v) && std::fabs(v) < 1e12 ? fmt("%.0f", v) : fmt("%g", v);
}

struct Flattener {
    const MsgSpec& spec;
    Decoded& out;
    size_t i = 0;                  // next spec entry
    bool overrun = false;          // more leaves than specs: catalog out of date
    bool sender_pending = false;   // the next leaf is the sender id
    std::string* entry = nullptr;  // set while inside a List element
    std::string where;             // "peers[2]." while inside a List element
    std::string group;             // open "q.x" style group

    Flattener(const MsgSpec& s, Decoded& d) : spec(s), out(d) {}

    template <class T>
    void operator()(T& v) {
        if constexpr (codec::detail::has_fields<T, Flattener>::value) {
            v.fields(*this);
        } else if constexpr (std::is_floating_point_v<T>) {
            const FieldSpec* s = next();
            if (s) leaf(*s, static_cast<double>(v), true, fmt("%.*f", s->decimals, static_cast<double>(v)));
        } else if constexpr (std::is_integral_v<T>) {
            const FieldSpec* s = next();
            if (!s) return;
            std::string text = std::is_signed_v<T> ? std::to_string(static_cast<long long>(v))
                                                   : std::to_string(static_cast<unsigned long long>(v));
            if (s->kind == Kind::Hex) text = fmt("0x%0*llX", int(2 * sizeof(T)), static_cast<unsigned long long>(v));
            leaf(*s, static_cast<double>(v), false, std::move(text));
        } else {
            static_assert(sizeof(T) == 0, "catalog: unsupported field type");
        }
    }

    template <class T, size_t N>
    void operator()(wirelink::List<T, N>& list) {
        const FieldSpec* s = next();
        if (!s) return;
        const size_t first = i;
        LeafCounter c;
        T probe{};
        c(probe);
        std::string text = std::to_string(list.count);
        for (size_t k = 0; k < list.count; ++k) {
            i = first;
            std::string e;
            entry = &e;
            where = std::string(s->name) + "[" + std::to_string(k) + "].";
            (*this)(list.items[k]);
            text += " [" + e + "]";
        }
        entry = nullptr;
        where.clear();
        i = first + c.n;
        out.fields.push_back({s->name, text});
    }

    const FieldSpec* next() {
        if (i >= spec.fields.size()) {
            overrun = true;
            return nullptr;
        }
        if (static_cast<int>(i) == spec.sender_leaf) sender_pending = true;
        return &spec.fields[i++];
    }

    void leaf(const FieldSpec& s, double v, bool is_float, std::string text) {
        if (sender_pending) {
            out.sender = static_cast<int>(v);
            sender_pending = false;
        }
        check(s, v, is_float);
        text = shown(s, v, std::move(text));
        const char* unit = is_float && !std::isfinite(v) ? "" : s.unit;  // "nan", not "nandBm"

        if (entry) {
            if (!entry->empty()) *entry += ',';
            *entry += text + unit;
            return;
        }
        const char* dot = std::strchr(s.name, '.');
        if (!dot) {
            group.clear();
            out.fields.push_back({s.name, text + unit});
            return;
        }
        // q.x q.y q.z q.w -> q=(x,y,z,w)unit
        const std::string g(s.name, dot);
        if (group == g) {
            out.fields.back().value += "," + text;
        } else {
            group = g;
            out.fields.push_back({g, "(" + text});
        }
        const bool last = i >= spec.fields.size() ||
                          std::strncmp(spec.fields[i].name, s.name, g.size() + 1) != 0;
        if (last) {
            out.fields.back().value += std::string(")") + unit;
            group.clear();
        }
    }

    static std::string shown(const FieldSpec& s, double v, std::string text) {
        const uint8_t raw = u8(v);
        switch (s.kind) {
            case Kind::Lat:
            case Kind::Lon: return fmt("%.7f", v / 1e7);
            case Kind::RxId: return raw == ids::BROADCAST_ID ? "ALL" : text;
            case Kind::Mode: return names::find_mode(raw) ? names::find_mode(raw)->name : text + "?";
            case Kind::Armed: return raw <= 1 ? names::armed_name(raw) : text + "?";
            case Kind::Gate: return raw <= 1 ? names::gate_name(raw) : text + "?";
            case Kind::Faults: {
                const auto flags = static_cast<uint16_t>(v);
                if (flags == 0) return "none";
                std::string out;
                for (const auto& n : names::fault_names(flags)) out += (out.empty() ? "" : "|") + n;
                return out;
            }
            default: return text;
        }
    }

    void check(const FieldSpec& s, double v, bool is_float) {
        Problems& p = out.problems;
        const std::string name = where + s.name;
        if (is_float && !std::isfinite(v)) {
            if (!(s.nan_ok && std::isnan(v))) error(p, name + (std::isnan(v) ? " is NaN" : " is inf"));
            return;
        }
        if (v < s.lo || v > s.hi) {
            const bool deg = s.kind == Kind::Lat || s.kind == Kind::Lon;
            const std::string val = name + " " + (deg ? fmt("%.7f", v / 1e7) : bound(v)) + s.unit;
            if (std::isinf(s.hi)) error(p, val + " < " + bound(s.lo));
            else if (std::isinf(s.lo)) error(p, val + " > " + bound(s.hi));
            else error(p, val + " out of range [" + bound(deg ? s.lo / 1e7 : s.lo) + "," + bound(deg ? s.hi / 1e7 : s.hi) + "]");
        }
        const uint8_t raw = u8(v);
        const bool boat = raw >= ids::BOAT_ID_MIN && raw <= ids::BOAT_ID_MAX;
        switch (s.kind) {
            case Kind::BoatId:
                if (!boat) error(p, fmt("%s %u is not a boat id", name.c_str(), unsigned(raw)));
                break;
            case Kind::RxId:
                if (!boat && raw != ids::BROADCAST_ID) error(p, fmt("%s %u is not a boat id", name.c_str(), unsigned(raw)));
                break;
            case Kind::PeerId:
                if (!boat && raw != ids::GROUND_STATION_ID) error(p, fmt("%s %u is not a boat id", name.c_str(), unsigned(raw)));
                break;
            case Kind::Mode:
                if (!names::find_mode(raw)) warn(p, fmt("unknown mode %u", unsigned(raw)));
                break;
            case Kind::Armed:
                if (raw > 1) error(p, fmt("%s %u is not an ArmedState", name.c_str(), unsigned(raw)));
                break;
            case Kind::Gate:
                if (raw > 1) error(p, fmt("%s %u is not a GateState", name.c_str(), unsigned(raw)));
                break;
            case Kind::Faults: {
                uint16_t known = 0;
                for (const auto& f : names::FAULTS) known |= f.bit;
                const auto bad = static_cast<uint16_t>(static_cast<uint16_t>(v) & ~known);
                if (bad) warn(p, fmt("%s has undefined bits 0x%04X", name.c_str(), unsigned(bad)));
                break;
            }
            default: break;
        }
    }
};

// ---------------------------------------------------------------------------
// Decoders
// ---------------------------------------------------------------------------

template <class T>
void no_check(const T&, Problems&) {}

template <class T, void (*Check)(const T&, Problems&) = no_check<T>>
void decode_as(const MsgSpec& spec, const wirelink::Frame& f, Decoded& out) {
    T m{};
    codec::Reader r{f.payload.data.data(), f.len};
    r(m);
    if (!r.ok || r.pos != f.len) {
        // The two sides disagree on the layout (or the frame is damaged in a
        // way the CRC missed): the values would be meaningless.
        error(out.problems, r.ok ? fmt("len %u, expected %zu (layout mismatch?)", unsigned(f.len), r.pos)
                                 : fmt("len %u too short or bad list count (layout mismatch?)", unsigned(f.len)));
        out.fields.push_back({"raw", hex(f.payload.data.data(), f.len)});
        return;
    }
    Flattener fl{spec, out};
    fl(m);
    if (fl.overrun || fl.i != spec.fields.size()) error(out.problems, "catalog out of date for this message");
    Check(m, out.problems);
}

// Upstream has not defined its fields yet: show the bytes, not a problem.
void decode_raw(const MsgSpec&, const wirelink::Frame& f, Decoded& out) {
    out.fields.push_back({"payload", f.len ? hex(f.payload.data.data(), f.len) : "(empty)"});
}

void check_fix(uint8_t fix_quality, int32_t lat, int32_t lon, Problems& p) {
    if (fix_quality > 0 && lat == 0 && lon == 0) error(p, "fix at 0,0");
}

void check_base_position(const base::BasePosition& m, Problems& p) {
    check_fix(m.fix_quality, m.lat, m.lon, p);
}

void check_gps(const ser::GPS& m, Problems& p) { check_fix(m.fix_quality, m.lat, m.lon, p); }

void check_attitude(const ser::Attitude& m, Problems& p) {
    const double n = std::sqrt(double(m.quat_x) * m.quat_x + double(m.quat_y) * m.quat_y +
                               double(m.quat_z) * m.quat_z + double(m.quat_w) * m.quat_w);
    // NaN components are already reported by the field check.
    if (std::isfinite(n) && (n < 0.9 || n > 1.1)) error(p, fmt("quat norm %.2f", n));
}

void check_config(const ser::Config& m, Problems& p) {
    if (!(m.pwm_min < m.pwm_neutral && m.pwm_neutral < m.pwm_max)) {
        error(p, fmt("pwm min<neutral<max violated (%u,%u,%u)", unsigned(m.pwm_min),
                     unsigned(m.pwm_neutral), unsigned(m.pwm_max)));
    }
    // Nominal LoRa bandwidths; firmware may round them (7.8125k vs 7.8k), so 1%.
    static const double BW[] = {7.8e3, 10.4e3, 15.6e3, 20.8e3, 31.25e3, 41.7e3, 62.5e3, 125e3, 250e3, 500e3};
    const bool ok = std::any_of(std::begin(BW), std::end(BW),
                                [&](double bw) { return std::fabs(m.lora_bw_hz - bw) <= 0.01 * bw; });
    if (!ok) error(p, fmt("lora_bw %u Hz is not a LoRa bandwidth", unsigned(m.lora_bw_hz)));
}

template <class T>
uint8_t type_of() { return static_cast<uint8_t>(T::TYPE); }

std::vector<MsgSpec> lora_messages() {
    std::vector<FieldSpec> self;
    peer_entry(self);
    return {
        {type_of<lora::Disable>(), "Disable", {of("rx_id", Kind::RxId)}, -1, decode_as<lora::Disable>},
        {type_of<lora::SelfStatus>(), "SelfStatus", self, 0, decode_as<lora::SelfStatus>},
        {type_of<lora::Command>(), "Command",
         {of("rx_id", Kind::RxId), num("lin", "mm/s"), num("ang", "mrad/s")}, -1, decode_as<lora::Command>},
        {type_of<lora::Status>(), "Status",
         {of("tx_id", Kind::BoatId), of("mode", Kind::Mode), of("armed", Kind::Armed), of("gate", Kind::Gate),
          of("faults", Kind::Faults), nan_ok(in(num("heading", "deg", 1), 0, 360)), nan_ok(rssi("gs_rssi")),
          nan_ok(snr("gs_snr"))},
         0, decode_as<lora::Status>},
        {type_of<lora::Reenable>(), "Reenable", {of("rx_id", Kind::RxId)}, -1, decode_as<lora::Reenable>},
        {type_of<lora::SetMode>(), "SetMode",
         {of("rx_id", Kind::RxId), of("mode", Kind::Mode), of("armed", Kind::Armed)}, -1,
         decode_as<lora::SetMode>},
        {type_of<base::RxInfo>(), "RxInfo", {rssi("rssi"), snr("snr")}, -1, decode_as<base::RxInfo>},
        {type_of<base::BaseStatus>(), "BaseStatus",
         {num("uptime", "ms"), num("rx_ok"), num("rx_bad"), num("tx")}, -1, decode_as<base::BaseStatus>},
        {type_of<base::BasePosition>(), "BasePosition",
         {lat(), lon(), num("fix"), in(num("sats"), 0, 64)}, -1,
         decode_as<base::BasePosition, check_base_position>},
    };
}

std::vector<MsgSpec> serial_messages() {
    using namespace boat::esc;
    std::vector<FieldSpec> broadcast{lat(), lon(), num("scalar")};
    return {
        {type_of<ser::Attitude>(), "Attitude",
         {num("q.x"), num("q.y"), num("q.z"), num("q.w"), num("gyro.x"), num("gyro.y"), num("gyro.z"),
          num("acc.x"), num("acc.y"), num("acc.z"), of("calib", Kind::Hex)},
         -1, decode_as<ser::Attitude, check_attitude>},
        {type_of<ser::GPS>(), "GPS",
         {lat(), lon(), num("utc", "ms"), num("fix"), in(num("sats"), 0, 64), in(num("hdop"), 0, HUGE_VAL),
          in(num("sog"), 0, HUGE_VAL), in(num("cog", "deg", 1), 0, 360)},
         -1, decode_as<ser::GPS, check_gps>},
        {type_of<ser::PeerTable>(), "PeerTable", with_list({}, "peers", peer_entry), -1,
         decode_as<ser::PeerTable>},
        // DS18B20 range.
        {type_of<ser::OwnScalar>(), "OwnScalar", {in(num("scalar", "°C"), -55, 125)}, -1,
         decode_as<ser::OwnScalar>},
        {type_of<ser::ReceivedCommand>(), "ReceivedCommand", {}, -1, decode_raw},
        {type_of<ser::Status>(), "Status",
         with_list({of("boat_id", Kind::BoatId), of("gate", Kind::Gate), of("faults", Kind::Faults)}, "links",
                   link_status),
         -1, decode_as<ser::Status>},
        {type_of<ser::MotorCommand>(), "MotorCommand",
         {in(num("port", "us"), PWM_MIN_US, PWM_MAX_US), in(num("stbd", "us"), PWM_MIN_US, PWM_MAX_US)}, -1,
         decode_as<ser::MotorCommand>},
        {type_of<ser::BroadcastPayload>(), "BroadcastPayload", broadcast, -1, decode_as<ser::BroadcastPayload>},
        {type_of<ser::Config>(), "Config",
         {of("boat_id", Kind::BoatId), num("slot"), in(num("sf"), 5, 12), num("bw", "Hz"),
          in(num("freq", "Hz"), 902e6, 928e6), num("pwm_min", "us"), num("pwm_neutral", "us"),
          num("pwm_max", "us"), in(num("output_enable"), 0, 1)},
         -1, decode_as<ser::Config, check_config>},
        {type_of<ser::Heartbeat>(), "Heartbeat", {of("ros_mode", Kind::Mode)}, -1, decode_as<ser::Heartbeat>},
    };
}

}  // namespace

bool Decoded::has(Severity s) const {
    return std::any_of(problems.begin(), problems.end(), [&](const Problem& p) { return p.severity == s; });
}

const char* proto_name(Proto p) { return p == Proto::Lora ? "lora" : "serial"; }

bool parse_proto(const std::string& s, Proto& out) {
    if (s == "lora") out = Proto::Lora;
    else if (s == "serial") out = Proto::Serial;
    else return false;
    return true;
}

const std::vector<MsgSpec>& messages(Proto p) {
    static const std::vector<MsgSpec> lora_set = lora_messages();
    static const std::vector<MsgSpec> serial_set = serial_messages();
    return p == Proto::Lora ? lora_set : serial_set;
}

const MsgSpec* find(Proto p, uint8_t type) {
    for (const auto& m : messages(p))
        if (m.type == type) return &m;
    return nullptr;
}

const MsgSpec* find(Proto p, const std::string& name) {
    auto lower = [](std::string s) {
        for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return s;
    };
    for (const auto& m : messages(p))
        if (lower(m.name) == lower(name)) return &m;
    return nullptr;
}

Decoded decode(Proto p, const wirelink::Frame& f) {
    Decoded out;
    const MsgSpec* spec = find(p, f.type);
    if (!spec) {
        out.name = fmt("type=0x%02X", unsigned(f.type));
        out.fields.push_back({"len", std::to_string(f.len)});
        out.fields.push_back({"", hex(f.payload.data.data(), f.len)});
        warn(out.problems, "unknown type");
        return out;
    }
    out.name = spec->name;
    out.known = true;
    spec->decode(*spec, f, out);
    return out;
}

std::string hex(const uint8_t* data, size_t len) {
    std::string s;
    for (size_t i = 0; i < len; ++i) s += fmt("%02x", data[i]);
    return s;
}

}  // namespace basestation::catalog
