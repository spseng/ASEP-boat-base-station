# Changes proposed for ASEP-boat

This repository never modifies `ASEP-boat` (it is a read-only submodule under
`external/ASEP-boat`). Anything the base station needs from the shared code
is listed here for you to apply upstream. Each item says what to change, why,
and what the base station does in the meantime.

Checked against ASEP-boat `4e07b3d` ("add mcu bridge msgs").

| § | Item | State |
|---|---|---|
| 1 | `wire.h`: `int16_t` and nested structs | **ADOPTED** in `7b59f1e` |
| 2 | `msg/lora.h`: a meaningful LoRa message set | **ADOPTED** in `7b59f1e` (upstream's own version) |
| 3 | Base-local messages (`>= 0x80`); what the base ESP32 (RadioGround) must do | **ADOPTED** in `2658776` (`wirelink/msg/base.h`); firmware behaviour **agreed**, still to implement |
| 4 | Adding modes | information |
| 5 | Minor items | open |
| 6 | Found in `firmware/lib/RadioBoat` (`4e07b3d`) | open |

---

## 1. `wire.h`: `int16_t` and nested structs: ADOPTED in `7b59f1e`

`Reader` / `Writer` now have `int16_t` overloads, and the deleted catch-all
template was replaced by one that recurses into `fields()`. Every LoRa
message, `serial::PeerTable` and the base station's own 0x80+ messages pack
and unpack with `wirelink::codec::pack` / `unpack`.

**Base station.** Its stop-gap codec (`src/proto/basestation/proto/codec.h`)
and the byte-compatibility test that guarded it are gone. Everything now
uses upstream's codec directly. `unpack` checks `r.pos == frame.len` just as
the old codec did, so a layout mismatch still fails to decode instead of
producing plausible garbage.

---

## 2. `msg/lora.h`: the LoRa message set: ADOPTED in `7b59f1e`

Upstream adopted its own version of the proposal, and the base station now
uses `wirelink::msg::lora` as is. Its proposal (`src/proto/basestation/proto/lora.h`)
is deleted. Compared with the proposal, `Status` was folded into
`SelfStatus`, which each boat sends once per TDMA cycle, and type 3 is now
`Heartbeat`:

| Type | Message |
|---|---|
| 0 | `Disable {rx_id}` |
| 1 | `SelfStatus {PeerEntry self, float heading_deg, uint8 state, uint8 faults, float gs_rssi, float gs_snr}` |
| 2 | `Command {rx_id, int16 lin_vel mm/s, int16 ang_vel mrad/s}` |
| 3 | `Heartbeat {}` |
| 4 | `Reenable {rx_id}` |
| 5 | `SetMode {rx_id, mode, armed}` |

`state` packs mode (bits 0-3), armed (bit 4) and gate enabled (bit 5);
bits 6-7 are reserved. Upstream `get_mode` / `get_armed` / `get_gate` /
`make_state` (`2cf1d1e`) read and write it; the base station uses them.
Encoded sizes are 1 / 27 / 5 / 0 / 1 / 3 bytes (SelfStatus lost `age_ms`
in `4e07b3d`), all well under `MAX_LORA_PAYLOAD = 64`.

Radio plan (agreed): SF6 / 500 kHz (`boat_defs/lora.h`), a 10 Hz cycle of
`NUM_IDS` = 4 slots of `TRANSMIT_INTERVAL` = 25 ms: the land station (id 0)
and 3 boats. Each boat sends one `SelfStatus` per cycle. In its slot the
land station (base ESP32) transmits exactly one frame, the one the app
left in its mailbox, or a `Heartbeat`. See §3 and the README.

Small follow-ups for `lora.h`, none of them blocking:

- **Document the NaN conventions** next to the fields: `heading_deg` NaN =
  no compass; `gs_rssi` / `gs_snr` NaN = this boat has not heard a land frame
  yet. The base station relies on both (shows course over ground, and `-`).
- **`Heartbeat::fields(F& f) {}`** triggers `-Wunused-parameter` in every
  translation unit that packs a Heartbeat. Drop the name: `fields(F&) {}`.
  Meanwhile the base station includes the wirelink and boat_defs headers as
  SYSTEM headers so upstream warnings don't break its zero-warning build.

Consequences on the boat side (not implemented here):

- The boat's ESP32 parses `Disable` itself. It must check `rx_id` against
  its own ID and against `BROADCAST_ID`.
- `Command`, `Reenable` and `SetMode` are passed up to the Pi as serial msg 5
  (`ReceivedCommand {lora_type, payload}`, done in `4e07b3d`; see §6 for
  the empty payload).
- `SetMode.mode` is a full byte, but `state` reports only 4 bits: a boat
  should refuse modes above 15. The base station's raw mode picker stops
  at 15.

---

## 3. Base-local messages (`>= 0x80`): ADOPTED in `2658776`

Now defined upstream in `wirelink/msg/base.h` (this repository's copy is
gone; it uses upstream's). These are emitted only by
the base-station ESP32 on its USB serial link and are never transmitted over
LoRa:

| Type | Message | When |
|---|---|---|
| `0x80` | `RxInfo {rssi, snr}` | Immediately after each relayed LoRa frame. The UI attaches it to the frame just before it. |
| `0x81` | `BaseStatus {uptime_ms, rx_ok, rx_bad, tx_count}` | About 1 Hz. `tx_count` = land frames transmitted (~10 per second with the TDMA plan). |
| `0x82` | `BasePosition {int32 lat, int32 lon (1e-7 deg), uint8 fix_quality (0 = no fix), uint8 satellites}` | About 1 Hz, only if the base ESP32 has a GPS. A fix overrides the position set by hand in the UI. |

All are optional; the UI still works if the ESP32 never sends them.
Upstream `base.h` documents the `>= 0x80` rule; a one-line note in `lora.h`
as well would stop a future LoRa type from being numbered into that range.

**Behaviour the base station expects from `firmware/src/ground`
(RadioGround): agreed design.** The ESP32 is a mailbox; every decision
about what a land slot carries is made by the app (`src/core/src/uplink.cpp`),
so RadioGround can stay a near copy of RadioBoat with `my_tx_id =
GROUND_STATION_ID`:

- **LoRa → USB:** write the received bytes to USB unchanged. They already
  are a wirelink packet (`00 … 00`, the boat's own `seq`, which the app
  uses to count loss). Then, optionally, `RxInfo {rssi, snr}`. Unwrapping
  is only needed to learn the sender for the TDMA schedule, exactly as
  RadioBoat does (`SelfStatus.self.id`).
- **USB → mailbox:** keep **one** frame, the latest one the app sent. A new
  one replaces it, whatever its type. No queues, no priorities. It is
  sensible to accept only LoRa land types (0, 2, 3, 4, 5) that fit
  `MAX_LORA_PAYLOAD`, and to ignore anything else.
- **Land slot (id 0, same `schedule_next_transmit()` as RadioBoat):**
  transmit the mailbox frame with the app's `seq` (`wrap(frame,
  frame.seq)`) and empty the mailbox; if it is empty, transmit a
  `Heartbeat` of its own. **Then write the transmitted packet to USB as
  well: the echo.** That is how the app learns that the slot happened and
  what went out. Emptying the mailbox matters: if the app stops, Commands
  stop, and the boat's command timeout stops the boat.
- **Start-up:** boats transmit only after they have heard something
  (`may_transmit`), so the ground ESP32 must transmit in its slot from
  `begin()` on (e.g. `may_transmit = true; my_next_transmit_us = micros();`).
  With no boat in range it then beacons every `NUM_IDS * TRANSMIT_INTERVAL`.

In pseudo-code, the only differences from RadioBoat:

```cpp
// receive(): after RadioBoat's sender bookkeeping
pcLink.write_raw(buf, len);           // relay unchanged
pcLink.send(RxInfo{rssi, snr});       // optional

// transmit():
auto f = pcLink.take_next();          // the mailbox; empties it
auto packet = f ? wrap(*f, f->seq) : wrap(heartbeat, hb_seq++);
sx1262.startTransmit(packet->data.data(), packet->len);
pcLink.write_raw(packet->data.data(), packet->len);   // the echo
```

What the app does on its side, for reference: Disable / Reenable / SetMode
are handed over one at a time and again after every slot whose echo shows
something else, until their own echo comes back (Disable: three slots).
While one is waiting, Commands are held back. Otherwise the latest Command
is refreshed 20 times a second. Firmware without the echo still works: the
app then counts each copy as unconfirmed after 250 ms and moves on.
`asep_fake_base` implements exactly this firmware (`--no-echo` for one
without the echo) and can serve as a reference.

`BaseStatus` / `BasePosition` are optional; the app works without them.
The UI's default serial rate is `boat::serial::BAUD_RATE` (115200); it can
be changed in the UI.

---

## 4. Modes: adding new ones (e.g. calibration)

`SetMode {rx_id, mode, armed}` and the mode bits of `SelfStatus.state` carry
the raw value of `boat::mode::Mode`, so adding a mode needs **no protocol
change**, as long as it stays below 16 (4 bits in `state`):

1. Add the value to `boat_defs/mode.h` upstream, e.g. `CALIBRATION = 4`.
2. Handle it on the boat (the Pi's `mode_manager`).
3. Add one line to `MODES` in `src/proto/basestation/proto/names.h` here
   (name + tooltip). It then shows up in the UI's mode picker.

Until step 3 is done the base station still works with the new mode: a
reported mode it doesn't know is shown as `MODE 4`, and the *Set mode*
picker has an "Other (raw value)" entry that sends any number 0–15.

Keep "armed" separate from mode, as it is now: a boat can be in
AUTONOMOUS but disarmed, which is the safe way to switch modes before
letting the thrusters run.

If a future mode needs parameters (e.g. which sensor to calibrate), that's
the point to add a dedicated message instead of stretching `SetMode`.

---

## 5. Minor / for information

- **Command timeout on the boat: at least 2-3 TDMA cycles (200-300 ms).**
  A `Command` only goes out when the land slot has nothing more urgent, and
  LoRa loses packets, so one missing cycle is normal. A timeout of a single
  cycle would make teleop stutter. The simulator uses 500 ms.
- **Serial `Config` LoRa fields vs `boat_defs/lora.h`.** `serial::Config`
  still carries `lora_sf`, `lora_bw_hz` and `lora_freq_hz`, while the fleet's
  settings are now compile-time constants in `boat_defs/lora.h` (in other
  units: MHz / kHz floats vs Hz integers). Two sources of truth can
  disagree, and a boat whose radio differs from the fleet's is simply
  inaudible. Recommended: remove the three fields from `Config` and have the
  ESP32 use `boat::lora::config`. If they must stay (e.g. for field tests),
  have the Pi fill them from `boat_defs/lora.h`. Meanwhile `wl_mon --proto
  serial` warns when they differ.
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
- **`Heartbeat`'s unused parameter** and the **0x80+ note in `lora.h`**:
  see §2 and §3.
- ~~`boat_defs/lora.h`: `FREQUENCY` comment said kHz~~. Fixed upstream in
  `4e07b3d` (MHz).
- ~~`wirelink/CMakeLists.txt` listed `src/parser.cpp`~~. Fixed upstream in
  `06bce72`.

---

## 6. Found in `firmware/lib/RadioBoat` (`4e07b3d`)

Read while designing §3; RadioGround will share most of this code.

- **`ReceivedCommand.payload` reaches the Pi empty.** `framing::unwrap`
  sets `Frame::len` but leaves `Frame::payload.len` at 0, so
  `command.payload = frame->payload;` packs a zero-length payload: the Pi
  gets the type of the Command / SetMode / Reenable but none of its fields.
  Fix: `command.payload.len = frame->len;` after the copy (or have
  `unwrap` set `payload.len`, see "Two lengths" in §5). `wl_mon --proto
  serial` decodes the LoRa message inside a ReceivedCommand and flags an
  empty one.
- **`wrap(frame, 0)`: every SelfStatus has seq 0.** The app counts loss per
  boat from `seq`; with a constant 0 every frame after the first looks like
  a duplicate and the loss shows 0 %. Use a counter, e.g. `static uint8_t
  seq; wrap(frame, seq++)`. (The Link panel marks a sender whose seq does
  not advance.)
- **`heading_deg`, `gs_rssi`, `gs_snr` are sent as `0.0` while
  unimplemented.** The app reads NaN as "unknown" (§2). With 0.0 it shows a
  compass heading of north and a land link of 0 dBm. Send `NAN` until they
  are filled in.
- **`RadioGround.h` defines `volatile bool Radio::rx_flag = false;` in the
  header.** Once both `RadioGround.cpp` and `main.cpp` include it, the link
  fails with a multiple definition. Keep the definition in the .cpp, as
  RadioBoat does.
- **`Disable` is still a TODO** in `receive()`. Until it is implemented the
  app's DISABLE does nothing on a real boat; only the hardware E-stop does.

