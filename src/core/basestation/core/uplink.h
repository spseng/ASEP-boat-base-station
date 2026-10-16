#pragma once

// Decides what goes into the base-station ESP32's single "next land frame".
//
// The ESP32 keeps only the latest frame the app gave it and transmits it in
// the land slot (or a Heartbeat of its own when it has none), then writes
// the transmitted packet back over USB: the echo. All the policy lives
// here, so the firmware stays a mailbox:
//
//   - One-shot frames (Disable, Reenable, SetMode) go out one at a time.
//     The frame at the head of the queue is handed to the ESP32 and handed
//     again after every land slot whose echo shows something else, until
//     its own echo comes back; a Disable needs `copies` echoes, i.e. that
//     many land slots. A Disable goes ahead of queued Reenable / SetMode.
//     Echoes are matched by content (type and payload), not seq: every
//     hand-over gets a new seq, and an older one may be what went out.
//   - While any one-shot frame is queued, Commands are held back: they
//     would overwrite it in the mailbox.
//   - Otherwise TeleopSender offers the latest Command at its own rate and
//     the ESP32 keeps the newest.
//
// Without echoes (firmware that does not echo, or a USB hiccup) a copy is
// counted as sent-but-unconfirmed after echo_timeout, so the queue still
// drains, at one copy per echo_timeout. A frame that cannot be sent at all
// within max_age (link closed) is dropped with an error event rather than
// going out long after the operator asked for it.

#include <basestation/core/common.h>
#include <basestation/core/event_log.h>
#include <basestation/core/link_session.h>
#include <basestation/proto/msgs.h>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace basestation {

class Uplink {
public:
    struct Config {
        std::chrono::milliseconds echo_timeout{250};  // > one TDMA cycle (100 ms)
        int max_attempts = 3;                         // slots carrying something else, per copy
        std::chrono::milliseconds max_age{3000};
    };

    struct Status {
        uint64_t slots = 0;           // land transmissions echoed by the ESP32
        uint64_t heartbeats = 0;      // ... of which the ESP32's own Heartbeat (mailbox empty)
        uint64_t commands_on_air = 0; // ... of which one of our Commands
        uint64_t oneshots_on_air = 0; // ... of which one of our one-shot frames
        Clock::time_point last_slot{};
        double slot_period_s = NAN;   // smoothed time between echoes
        size_t queued = 0;            // one-shot frames not finished yet
        uint64_t unconfirmed = 0;     // copies given up on without an echo
        uint64_t dropped = 0;         // frames dropped after max_age
        uint64_t commands_held = 0;   // offers refused because a one-shot was queued

        bool echoes_seen() const { return slots > 0; }
    };

    enum class Offer { Sent, Held, Failed };

    // Registers itself as `link`'s receive listener.
    explicit Uplink(LinkSession& link, EventLog* events = nullptr);
    ~Uplink();
    Uplink(const Uplink&) = delete;
    Uplink& operator=(const Uplink&) = delete;

    void set_config(const Config& cfg);
    Config config() const;

    // Queues a one-shot frame; `label` names it in events ("DISABLE to boat
    // 2"). Handed to the ESP32 at once if nothing is ahead of it. Returns
    // false if the link is closed (the frame stays queued for max_age, in
    // case it comes back).
    bool push(const wirelink::Frame& f, int copies, const std::string& label);
    template <class T>
    bool push_msg(const T& msg, int copies, const std::string& label) {
        wirelink::Frame f{};
        if (!wirelink::codec::pack(msg, f)) return false;
        return push(f, copies, label);
    }

    // Removes queued one-shot frames matching `pred` (e.g. the Disable
    // copies still owed to a boat that is being re-enabled).
    void drop_if(const std::function<bool(const wirelink::Frame&)>& pred);

    // Hands a Command to the ESP32 unless a one-shot frame is queued.
    Offer offer(const lora::Command& cmd);

    // Every frame received from the ESP32 (called on LinkSession's reader
    // thread; public for tests).
    void on_frame(const RxFrame& rx);

    Status status() const;

private:
    struct Item {
        wirelink::Frame frame;
        std::string label;
        int copies = 1;
        int confirmed = 0;
        int unconfirmed = 0;
        int attempts = 0;        // slots this copy missed
        bool in_mailbox = false; // handed to the ESP32, echo pending
        Clock::time_point handed{};  // last handed over ({} = never)
        Clock::time_point queued{};
    };

    void loop();
    // Hands the head to the ESP32 if it is not there yet. Caller holds mutex_.
    void hand_head(Clock::time_point now);
    void finish_copy(Item& it, bool confirmed);
    void pop_finished();

    LinkSession& link_;
    EventLog* events_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    Config cfg_{};
    Status status_{};
    std::deque<Item> queue_;
    bool quit_ = false;
    std::thread thread_;
};

// The land types the app hands to the ESP32 (and the ESP32 echoes):
// everything in lora:: except SelfStatus.
bool is_land_type(uint8_t type);

}  // namespace basestation
