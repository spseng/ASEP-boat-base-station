#pragma once

// Field codec for wirelink message structs.
//
// Every wirelink message exposes `template <class F> void fields(F& f)`, so
// any visitor can (de)serialise it. Upstream's wirelink::codec::Reader/Writer
// (wire.h) cannot yet handle int16_t fields or a nested struct field, both of
// which the LoRa messages use. This visitor is a drop-in superset of them:
// byte-for-byte identical output for everything upstream supports (verified
// by tests/test_codec.cpp), plus
//   - every integer width, signed and unsigned (int8..int64, uint8..uint64),
//     and enums (encoded as their underlying integer)
//   - any nested struct that itself has fields()
//   - wirelink::List<T, N> and wirelink::bytes::Payload, as upstream.
//
// Once upstream wire.h gains these overloads (see WIRELINK_CHANGES.md) this
// file can be replaced by wirelink::codec::pack/unpack.

#include <wirelink/framing.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace basestation::codec {

namespace detail {
    template <class T, class F, class = void>
    struct has_fields : std::false_type {};
    template <class T, class F>
    struct has_fields<T, F, std::void_t<decltype(std::declval<T&>().fields(std::declval<F&>()))>>
        : std::true_type {};
}

struct Reader {
    const uint8_t* buf;
    size_t len;
    size_t pos = 0;
    bool ok = true;

    template <class T>
    void operator()(T& v) {
        if constexpr (std::is_same_v<T, bool>) {
            v = get(1) != 0;
        } else if constexpr (std::is_integral_v<T>) {
            using U = std::make_unsigned_t<T>;
            v = static_cast<T>(static_cast<U>(get(sizeof(T))));
        } else if constexpr (std::is_enum_v<T>) {
            using U = std::make_unsigned_t<std::underlying_type_t<T>>;
            v = static_cast<T>(static_cast<U>(get(sizeof(T))));
        } else if constexpr (std::is_same_v<T, float>) {
            const uint32_t u = static_cast<uint32_t>(get(4));
            std::memcpy(&v, &u, sizeof u);
        } else if constexpr (std::is_same_v<T, double>) {
            const uint64_t u = get(8);
            std::memcpy(&v, &u, sizeof u);
        } else if constexpr (detail::has_fields<T, Reader>::value) {
            v.fields(*this);
        } else {
            static_assert(sizeof(T) == 0, "codec::Reader: unsupported field type");
        }
    }

    void operator()(wirelink::bytes::Payload& p) {
        uint8_t n = 0;
        (*this)(n);
        if (!ok || n > p.data.size()) { ok = false; p.len = 0; return; }
        p.len = n;
        for (size_t i = 0; i < p.len; ++i) (*this)(p.data[i]);
    }

    template <class T, size_t N>
    void operator()(wirelink::List<T, N>& list) {
        (*this)(list.count);
        if (!ok || list.count > N) { ok = false; list.count = 0; return; }
        for (size_t i = 0; i < list.count; ++i) (*this)(list.items[i]);
    }

private:
    uint64_t get(size_t n) {
        if (!ok || pos + n > len) { ok = false; return 0; }
        uint64_t v = 0;
        for (size_t i = 0; i < n; ++i) v |= static_cast<uint64_t>(buf[pos++]) << (8 * i);
        return v;
    }
};

struct Writer {
    uint8_t* buf;
    size_t cap;
    size_t pos = 0;
    bool ok = true;

    // Takes a non-const reference because fields() is non-const upstream.
    template <class T>
    void operator()(T& v) {
        if constexpr (std::is_same_v<T, bool>) {
            put(v ? 1 : 0, 1);
        } else if constexpr (std::is_integral_v<T>) {
            using U = std::make_unsigned_t<T>;
            put(static_cast<U>(v), sizeof(T));
        } else if constexpr (std::is_enum_v<T>) {
            using U = std::make_unsigned_t<std::underlying_type_t<T>>;
            put(static_cast<U>(v), sizeof(T));
        } else if constexpr (std::is_same_v<T, float>) {
            uint32_t u;
            std::memcpy(&u, &v, sizeof u);
            put(u, 4);
        } else if constexpr (std::is_same_v<T, double>) {
            uint64_t u;
            std::memcpy(&u, &v, sizeof u);
            put(u, 8);
        } else if constexpr (detail::has_fields<T, Writer>::value) {
            v.fields(*this);
        } else {
            static_assert(sizeof(T) == 0, "codec::Writer: unsupported field type");
        }
    }

    void operator()(wirelink::bytes::Payload& p) {
        if (p.len > p.data.size()) { ok = false; return; }
        uint8_t n = static_cast<uint8_t>(p.len);
        (*this)(n);
        for (size_t i = 0; i < p.len; ++i) (*this)(p.data[i]);
    }

    template <class T, size_t N>
    void operator()(wirelink::List<T, N>& list) {
        if (list.count > N) { ok = false; return; }
        (*this)(list.count);
        for (size_t i = 0; i < list.count; ++i) (*this)(list.items[i]);
    }

private:
    void put(uint64_t v, size_t n) {
        if (!ok || pos + n > cap) { ok = false; return; }
        for (size_t i = 0; i < n; ++i) buf[pos++] = static_cast<uint8_t>(v >> (8 * i));
    }
};

// Serialise `msg` into `frame` (type, len, payload). seq is assigned when the
// frame is sent. Returns false if the payload does not fit.
template <class T>
bool pack(T msg, wirelink::Frame& frame) {
    Writer w{frame.payload.data.data(), frame.payload.data.size()};
    w(msg);
    if (!w.ok) return false;
    frame.type = static_cast<uint8_t>(T::TYPE);
    frame.len = static_cast<uint8_t>(w.pos);
    frame.payload.len = w.pos;
    frame.seq = 0;
    frame.crc = 0;
    return true;
}

// Deserialise a frame into `out`. Fails on a type mismatch, a short payload,
// or trailing bytes (a length mismatch means the two sides disagree on the
// layout, which must not decode as plausible garbage).
template <class T>
bool unpack(const wirelink::Frame& frame, T& out) {
    if (frame.type != static_cast<uint8_t>(T::TYPE)) return false;
    Reader r{frame.payload.data.data(), frame.len};
    r(out);
    return r.ok && r.pos == frame.len;
}

// Encoded size of a message, in bytes on the wire (not sizeof).
template <class T>
size_t wire_size(T msg) {
    uint8_t scratch[wirelink::MAX_PAYLOAD];
    Writer w{scratch, sizeof scratch};
    w(msg);
    return w.ok ? w.pos : 0;
}

}  // namespace basestation::codec
