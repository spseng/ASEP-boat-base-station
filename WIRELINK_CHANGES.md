# Changes proposed for ASEP-boat

This repository never modifies `ASEP-boat` (it is a read-only submodule under
`external/ASEP-boat`). Anything the base station needs from the shared code
is listed here for you to apply upstream. Each item says what to change, why,
and what the base station does in the meantime.

Checked against ASEP-boat `06bce72` ("update cmake mislink").

---

## 1. `wire.h`: `Reader`/`Writer` cannot encode `int16_t` or nested structs (blocking for `lora.h`)

**Problem.** `wirelink::codec::Reader` and `Writer` only have overloads for
`uint8/16/32/64`, `int32`, `float`, `bytes::Payload` and `List<T, N>`; every
other type hits the deleted catch-all template. The draft `lora.h` uses both
missing cases:

- `Command::lin_vel` / `ang_vel` are `int16_t`, which does not compile.
- `SelfStatus` calls `f(self)` on a `PeerEntry`, and `Status` calls
  `f(links)` on a bare `LinkStatus`. Nested structs are only handled inside
  a `List`.

Nothing upstream calls `pack`/`unpack` on a LoRa message yet, so the build
doesn't break. It will as soon as the firmware or the Pi packs one.

**Suggested fix.** Add an `int16_t` overload, and **replace** the deleted
catch-all template with one that recurses into `fields()`:

```cpp
// Reader: replace  template <class T> void operator()(T&) = delete;
void operator()(int16_t& v) { v = static_cast<int16_t>(static_cast<uint16_t>(get(2))); }
// Any nested message struct (has fields()); other types still fail to compile here.
template <class T> void operator()(T& v) { v.fields(*this); }

// Writer: replace  template <class T> void operator()(T) = delete;
void operator()(int16_t v) { put(static_cast<uint16_t>(v), 2); }
template <class T> void operator()(T& v) { v.fields(*this); }
```

Replace the deleted template rather than adding a second one next to it: a
`T&` template and a `T` template side by side are ambiguous for lvalues. I
compiled this patch against `06bce72`'s `wire.h`. The draft
`lora::Command`, `SelfStatus` and `Status` and `serial::PeerTable` all pack,
and `int16_t -1234` and nested negative `int32` values round-trip. An
unsupported type such as `double` still fails at compile time.

**Meanwhile.** `src/proto/basestation/proto/codec.h` is a superset visitor
used by the base station. `tests/test_codec.cpp` checks that it produces
byte-for-byte the same output as upstream's `Writer` for every message
upstream can encode, so the two stay compatible.

---

## 2. `msg/lora.h`: adopt the revised message set

The draft was a placeholder, and you asked me to make it meaningful. The
revised set is in `src/proto/basestation/proto/lora.h`. To adopt it, copy it
over `wirelink/msg/lora.h` and rename the namespace
`basestation::lora` → `wirelink::msg::lora`.

| Type | Message | Change from the draft |
|---|---|---|
| 0 | `Disable {rx_id}` | Now carries `rx_id` (255 = every boat). The draft had no struct. |
| 1 | `SelfStatus {PeerEntry self}` | Unchanged. |
| 2 | `Command {rx_id, lin_vel, ang_vel}` | `age_ms` dropped (nobody knew what it was for; `seq` already exposes stale or missing commands). Units are **mm/s** and **mrad/s** per `boat_defs/units.h`; the draft's comments said m/s and rad/s. `+ang_vel` = counter-clockwise (REP 103). |
| 3 | `Status {tx_id, mode, armed, gate_state, fault_flags, heading_deg, gs_rssi, gs_snr}` | Rewritten. The draft's single `LinkStatus` and `age_ms` are replaced by what the ground station displays. `gs_rssi`/`gs_snr` are how well the boat hears the ground station, which is the teleop link margin. `heading_deg` is **an addition beyond what we discussed**. Drop it if you'd rather not spend 4 bytes; the UI falls back to course derived from the track. |
| 4 | `Reenable {rx_id}` | New. The boat's ESP32 passes it up to the Pi, which decides and tells its ESP32 to re-enable. |
| 5 | `SetMode {rx_id, mode, armed}` | New. Enums from `boat_defs/mode.h`. Handled by the Pi. |

Encoded sizes: 1 / 17 / 5 / 18 / 1 / 3 bytes. All are well under
`MAX_LORA_PAYLOAD = 64` (checked in `tests/test_codec.cpp`).

Consequences on the boat side (not implemented here):
- The boat's ESP32 parses `Disable` itself. It must check `rx_id` against
  its own ID and against `BROADCAST_ID`.
- `Command`, `Reenable` and `SetMode` are passed up to the Pi as serial msg 5
  (`ReceivedCommand`, still a TODO upstream).
- Each boat sends `Status` at a low rate (the simulator uses 1 Hz).

---

## 3. Reserve frame types `>= 0x80` for the PC ↔ base-station link

Defined in `src/proto/basestation/proto/base.h`. These are emitted only by
the base-station ESP32 on its USB serial link and are never transmitted over
LoRa:

| Type | Message | When |
|---|---|---|
| `0x80` | `RxInfo {rssi, snr}` | Immediately after each relayed LoRa frame. The UI attaches it to the frame just before it. |
| `0x81` | `BaseStatus {uptime_ms, rx_ok, rx_bad, tx_count}` | About 1 Hz. |

Both are optional; the UI still works if the ESP32 never sends them.
If you agree, add a comment to `lora.h` (or `framing.h`) reserving
`0x80–0xFF` so a future LoRa type never collides.

**Behaviour the base station expects from `firmware/src/ground`.** It relays
wirelink frames between USB serial and LoRa without changing them, in both
directions. The `seq` numbers stay the sender's own; the UI counts loss per
boat and per type from them. The UI's default serial rate is
`boat::serial::BAUD_RATE` (115200); it can be changed in the UI.

---

## 4. Minor / for information

- **`Link::feed` cannot report corrupt frames.** It returns `nullopt` for
  both "frame not finished" and "frame failed CRC/COBS", so a receiver can't
  count bad frames. The base station uses its own splitter
  (`src/core/src/frame_splitter.cpp`) around `framing::unwrap`. Optional
  upstream fix: return a small status enum, or keep a counter.
- **The CRC_EXTRA layout hash** described in SOFTWARE_ARCHITECTURE.md is not
  implemented yet (`crc16` starts from `0xFFFF`). The base station uses
  upstream `framing.cpp`, so it picks up the change automatically when the
  submodule is bumped. No action needed here.
- **Two lengths.** `Frame::len` and `Frame::payload.len` both exist.
  `codec::pack` sets only `frame.len`, and `framing::wrap` reads only
  `frame.len`. It works, but it's easy to set the wrong one. Consider
  dropping one.
- ~~`wirelink/CMakeLists.txt` listed `src/parser.cpp`~~. Fixed upstream in
  `06bce72`.
