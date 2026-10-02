#pragma once

// Message catalog for debugging tools (wl_mon): names every message of a
// wirelink link, decodes a frame into (field, value) text and flags values
// that cannot be right, so firmware sending garbage is easy to spot.
//
// Two message sets share the frame format but reuse type numbers, so the
// caller says which link it is looking at:
//   Proto::Lora    basestation::lora (0-5) + basestation::base (0x80+):
//                  what the base-station ESP32 sends the laptop
//   Proto::Serial  wirelink::msg::serial (1-10): the boat's Pi <-> ESP32 link
//
// Upstream fields() visitors pass values only, so field names live here,
// one spec per message in fields() order. tests/test_msg_catalog.cpp counts
// each struct's leaves against its spec, so a struct changed upstream fails
// the tests instead of silently mislabelling fields.

#include <basestation/proto/codec.h>

#include <wirelink/framing.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace basestation::catalog {

enum class Proto { Lora, Serial };

const char* proto_name(Proto p);
bool parse_proto(const std::string& s, Proto& out);

// Error: the value cannot be right ("BOGUS"). Warning: suspicious, or
// possibly just newer than this build (an unknown mode or type).
enum class Severity { Warning, Error };

struct Problem {
    Severity severity;
    std::string text;
};

struct Field {
    std::string name;
    std::string value;  // formatted, with unit where known
};

struct Decoded {
    std::string name;  // "Attitude", or "type=0x2A" when unknown
    bool known = false;
    std::vector<Field> fields;
    std::vector<Problem> problems;
    int sender = -1;  // boat id, for messages that carry their sender

    bool has(Severity s) const;
};

// How a leaf field is shown and checked.
enum class Kind : uint8_t {
    Num,     // number, optionally range-checked
    Hex,     // integer shown as hex
    BoatId,  // a boat: BOAT_ID_MIN..BOAT_ID_MAX
    RxId,    // a boat or BROADCAST_ID
    PeerId,  // a boat or GROUND_STATION_ID
    Lat,     // int32, 1e-7 deg
    Lon,
    Mode,    // boat::mode::Mode (unknown = warning: may be newer firmware)
    Armed,   // boat::mode::ArmedState
    Gate,    // boat::mode::GateState
    Faults,  // boat::mode::fault bits (undefined bits = warning)
    List,    // wirelink::List: this entry, then the element's leaves
};

struct FieldSpec {
    const char* name;  // "q.x" style names are shown grouped: q=(x,y,z,w)
    Kind kind = Kind::Num;
    const char* unit = "";
    int decimals = 2;         // floats only
    double lo = -HUGE_VAL;    // outside [lo, hi] is an error
    double hi = HUGE_VAL;
    bool nan_ok = false;      // NaN documented as "unknown"
};

struct MsgSpec {
    uint8_t type;
    const char* name;
    std::vector<FieldSpec> fields;  // leaves in fields() order
    int sender_leaf;                // index of the sender id leaf, or -1
    void (*decode)(const MsgSpec&, const wirelink::Frame&, Decoded&);
};

// Every message of `p`, sorted by type.
const std::vector<MsgSpec>& messages(Proto p);
const MsgSpec* find(Proto p, uint8_t type);
const MsgSpec* find(Proto p, const std::string& name);  // case-insensitive

Decoded decode(Proto p, const wirelink::Frame& f);

std::string hex(const uint8_t* data, size_t len);

// Counts leaves the way MsgSpec::fields lays them out: nested structs are
// flattened, a List is one leaf (its count) plus one element's leaves.
struct LeafCounter {
    size_t n = 0;

    template <class T>
    void operator()(T& v) {
        if constexpr (codec::detail::has_fields<T, LeafCounter>::value) {
            v.fields(*this);
        } else {
            ++n;
        }
    }

    template <class T, size_t N>
    void operator()(wirelink::List<T, N>&) {
        ++n;
        T item{};
        (*this)(item);
    }
};

template <class T>
size_t leaf_count() {
    T m{};
    LeafCounter c;
    c(m);
    return c.n;
}

}  // namespace basestation::catalog
