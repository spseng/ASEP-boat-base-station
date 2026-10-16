// Uplink: what the app hands to the base-station ESP32's single mailbox.
// Echoes are injected with on_frame() (what LinkSession's reader thread
// does), so the order of events is fixed and no test depends on timing,
// except the echo_timeout / max_age ones, which only use lower bounds.

#include <catch2/catch_test_macros.hpp>

#include <basestation/core/link_session.h>
#include <basestation/core/uplink.h>
#include <basestation/proto/msgs.h>

#include <boat_defs/ids.h>

#include <wirelink/wire.h>

#include "pty_peer.h"

#include <cmath>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

using namespace basestation;
using basestation::test::PtyPeer;
using basestation::test::open_link;
using namespace std::chrono_literals;

namespace {

size_t count_events(const EventLog& log, EventLevel level, const std::string& text) {
    size_t n = 0;
    for (const auto& e : log.snapshot())
        if (e.level == level && e.text.find(text) != std::string::npos) ++n;
    return n;
}

// No timeouts unless a test asks for them.
Uplink::Config slow() {
    Uplink::Config c;
    c.echo_timeout = 60s;
    c.max_age = 60s;
    return c;
}

template <class T>
wirelink::Frame frame_of(const T& msg) {
    wirelink::Frame f{};
    REQUIRE(wirelink::codec::pack(msg, f));
    return f;
}

// The ESP32 transmitted `f` in its land slot.
void echo(Uplink& up, const wirelink::Frame& f) { up.on_frame(RxFrame{Clock::now(), f}); }
void heartbeat(Uplink& up) { echo(up, frame_of(lora::Heartbeat{})); }

template <class T>
bool is(const wirelink::Frame& f, uint8_t rx_id) {
    T m{};
    return f.type == static_cast<uint8_t>(T::TYPE) && wirelink::codec::unpack(f, m) && m.rx_id == rx_id;
}

// Exactly one frame handed over (anything else would have arrived by then).
wirelink::Frame one(PtyPeer& pty) {
    const auto rx = pty.read(1, 1000ms);
    REQUIRE(rx.size() == 1);
    REQUIRE(pty.read_for(30ms).empty());
    return rx[0].frame;
}

template <class Pred>
bool wait_for(Pred done, std::chrono::milliseconds timeout = 3000ms) {
    const auto deadline = Clock::now() + timeout;
    while (!done() && Clock::now() < deadline) std::this_thread::sleep_for(2ms);
    return done();
}

}  // namespace

TEST_CASE("Uplink: land types", "[uplink]") {
    CHECK(is_land_type(static_cast<uint8_t>(lora::MsgType::Disable)));
    CHECK(is_land_type(static_cast<uint8_t>(lora::MsgType::Command)));
    CHECK(is_land_type(static_cast<uint8_t>(lora::MsgType::Heartbeat)));
    CHECK(is_land_type(static_cast<uint8_t>(lora::MsgType::Reenable)));
    CHECK(is_land_type(static_cast<uint8_t>(lora::MsgType::SetMode)));
    CHECK_FALSE(is_land_type(static_cast<uint8_t>(lora::MsgType::SelfStatus)));
    CHECK_FALSE(is_land_type(static_cast<uint8_t>(base::MsgType::RxInfo)));
}

TEST_CASE("Uplink: a one-shot frame holds Commands back until its echo", "[uplink][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    Uplink up(link);
    up.set_config(slow());

    CHECK(up.offer(lora::Command{1, 100, 0}) == Uplink::Offer::Sent);
    CHECK(is<lora::Command>(one(pty), 1));

    REQUIRE(up.push_msg(lora::Reenable{2}, 1, "REENABLE to boat 2"));
    const wirelink::Frame handed = one(pty);
    CHECK(is<lora::Reenable>(handed, 2));
    CHECK(up.status().queued == 1);

    // Would overwrite the Reenable in the mailbox.
    CHECK(up.offer(lora::Command{1, 200, 0}) == Uplink::Offer::Held);
    CHECK(up.status().commands_held == 1);

    // A slot passes with something else (the Command handed before): the
    // Reenable is handed again.
    echo(up, frame_of(lora::Command{1, 100, 0}));
    CHECK(is<lora::Reenable>(one(pty), 2));
    CHECK(up.status().commands_on_air == 1);

    echo(up, handed);
    CHECK(pty.read_for(30ms).empty());  // done: nothing more to hand over
    const auto st = up.status();
    CHECK(st.queued == 0);
    CHECK(st.oneshots_on_air == 1);
    CHECK(st.unconfirmed == 0);
    CHECK(st.slots == 2);
    CHECK(up.offer(lora::Command{1, 300, 0}) == Uplink::Offer::Sent);
    CHECK(is<lora::Command>(one(pty), 1));
}

TEST_CASE("Uplink: a Disable goes out in `copies` land slots", "[uplink][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    Uplink up(link);
    up.set_config(slow());

    REQUIRE(up.push_msg(lora::Disable{3}, 3, "DISABLE to boat 3"));
    for (int copy = 0; copy < 3; ++copy) {
        INFO("copy " << copy);
        const wirelink::Frame f = one(pty);
        CHECK(is<lora::Disable>(f, 3));
        CHECK(up.status().queued == 1);
        echo(up, f);
    }
    CHECK(pty.read_for(30ms).empty());
    CHECK(up.status().queued == 0);
    CHECK(up.status().oneshots_on_air == 3);
}

TEST_CASE("Uplink: idle slots are Heartbeats; a missed copy is handed again, then given up", "[uplink][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    EventLog log;
    Uplink up(link, &log);
    Uplink::Config cfg = slow();
    cfg.max_attempts = 3;
    up.set_config(cfg);

    heartbeat(up);
    CHECK(up.status().heartbeats == 1);
    CHECK(pty.read_for(30ms).empty());

    REQUIRE(up.push_msg(lora::SetMode{1, 1, 1}, 1, "SET_MODE to boat 1"));
    CHECK(is<lora::SetMode>(one(pty), 1));
    heartbeat(up);  // lost on USB, or overwritten: the slot carried a Heartbeat
    CHECK(is<lora::SetMode>(one(pty), 1));
    heartbeat(up);
    CHECK(is<lora::SetMode>(one(pty), 1));
    heartbeat(up);  // third miss: give up
    CHECK(pty.read_for(30ms).empty());
    const auto st = up.status();
    CHECK(st.queued == 0);
    CHECK(st.unconfirmed == 1);
    CHECK(st.heartbeats == 4);
    CHECK(count_events(log, EventLevel::Warn, "SET_MODE to boat 1: 1 of 1 copy not confirmed") == 1);
}

TEST_CASE("Uplink: a Disable goes ahead of queued Reenable / SetMode", "[uplink][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    Uplink up(link);
    up.set_config(slow());

    REQUIRE(up.push_msg(lora::SetMode{1, 1, 1}, 1, "SET_MODE"));
    const wirelink::Frame set_mode = one(pty);
    REQUIRE(up.push_msg(lora::Reenable{2}, 1, "REENABLE"));
    CHECK(pty.read_for(30ms).empty());  // queued behind the SetMode

    // Replaces the SetMode in the mailbox at once.
    REQUIRE(up.push_msg(lora::Disable{3}, 2, "DISABLE"));
    const wirelink::Frame disable = one(pty);
    CHECK(is<lora::Disable>(disable, 3));

    SECTION("the SetMode never made it: it is handed again after the Disable") {
        echo(up, disable);
        CHECK(is<lora::Disable>(one(pty), 3));
        echo(up, disable);
        CHECK(is<lora::SetMode>(one(pty), 1));
        echo(up, set_mode);
        CHECK(is<lora::Reenable>(one(pty), 2));
    }
    SECTION("the SetMode went out before it was overwritten: it is not sent twice") {
        echo(up, set_mode);
        CHECK(is<lora::Disable>(one(pty), 3));  // the slot missed the Disable
        echo(up, disable);
        CHECK(is<lora::Disable>(one(pty), 3));
        echo(up, disable);
        CHECK(is<lora::Reenable>(one(pty), 2));
    }
    CHECK(up.status().queued == 1);
}

TEST_CASE("Uplink: drop_if removes queued frames and hands the next", "[uplink][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    Uplink up(link);
    up.set_config(slow());

    REQUIRE(up.push_msg(lora::Disable{2}, 5, "DISABLE"));
    CHECK(is<lora::Disable>(one(pty), 2));
    REQUIRE(up.push_msg(lora::SetMode{2, 0, 0}, 1, "SET_MODE"));
    up.drop_if([](const wirelink::Frame& f) { return f.type == static_cast<uint8_t>(lora::MsgType::Disable); });
    CHECK(is<lora::SetMode>(one(pty), 2));
    CHECK(up.status().queued == 1);
}

TEST_CASE("Uplink: echoes arrive through the serial link", "[uplink][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    Uplink up(link);
    up.set_config(slow());

    REQUIRE(up.push_msg(lora::Disable{1}, 2, "DISABLE"));
    for (int copy = 0; copy < 2; ++copy) {
        const auto rx = pty.read(1, 1000ms);
        REQUIRE(rx.size() == 1);
        pty.write(rx[0].frame, rx[0].frame.seq);  // the ESP32 echoes it, seq and all
    }
    CHECK(wait_for([&] { return up.status().queued == 0; }));
    CHECK(up.status().slots == 2);
    // The echoes still reach poll(), for the fleet model and the log.
    std::vector<RxFrame> frames;
    CHECK(wait_for([&] {
        link.poll(frames);
        return frames.size() == 2;
    }));
}

TEST_CASE("Uplink: without echoes each copy times out as unconfirmed", "[uplink][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    EventLog log;
    Uplink up(link, &log);
    Uplink::Config cfg = slow();
    cfg.echo_timeout = 40ms;
    up.set_config(cfg);

    const auto t0 = Clock::now();
    REQUIRE(up.push_msg(lora::Disable{2}, 3, "DISABLE to boat 2"));
    const auto rx = pty.read(3, 3000ms);
    REQUIRE(rx.size() == 3);
    // Copy k is handed over no earlier than k timeouts after the first.
    CHECK(rx[1].t - t0 >= 40ms);
    CHECK(rx[2].t - t0 >= 80ms);
    for (const auto& r : rx) CHECK(is<lora::Disable>(r.frame, 2));
    CHECK(wait_for([&] { return up.status().queued == 0; }));
    CHECK(pty.read_for(60ms).empty());
    CHECK(up.status().unconfirmed == 3);
    CHECK(count_events(log, EventLevel::Warn, "DISABLE to boat 2: 3 of 3 copies not confirmed") == 1);
}

TEST_CASE("Uplink: a closed link drops one-shot frames after max_age", "[uplink]") {
    LinkSession link;  // never opened
    EventLog log;
    Uplink up(link, &log);
    Uplink::Config cfg = slow();
    cfg.echo_timeout = 10ms;
    cfg.max_age = 50ms;
    up.set_config(cfg);

    CHECK_FALSE(up.push_msg(lora::Reenable{1}, 1, "REENABLE to boat 1"));
    CHECK(up.offer(lora::Command{1, 0, 0}) == Uplink::Offer::Held);
    REQUIRE(wait_for([&] { return up.status().dropped == 1; }));
    CHECK(up.status().queued == 0);
    CHECK(count_events(log, EventLevel::Error, "REENABLE to boat 1: not sent") == 1);
    CHECK(up.offer(lora::Command{1, 0, 0}) == Uplink::Offer::Failed);
}

TEST_CASE("Uplink: period statistics and reset", "[uplink]") {
    LinkSession link;  // closed: echoes need no link
    Uplink up(link);
    const auto t0 = Clock::now();
    const wirelink::Frame hb = frame_of(lora::Heartbeat{});
    for (int ms : {0, 100, 200, 310, 400, 403}) up.on_frame(RxFrame{t0 + std::chrono::milliseconds(ms), hb});

    Uplink::Status st = up.status();
    CHECK(st.slots == 6);
    CHECK(st.heartbeats == 6);
    REQUIRE(std::isfinite(st.period_min_s));
    CHECK(std::fabs(st.period_min_s - 0.003) < 1e-6);  // back-to-back slots show up here...
    CHECK(std::fabs(st.period_max_s - 0.110) < 1e-6);
    CHECK(st.slot_period_s > 0.09);  // ...but not in the smoothed period
    CHECK(st.slot_period_s < 0.11);

    up.reset_stats();
    st = up.status();
    CHECK(st.slots == 0);
    CHECK(st.heartbeats == 0);
    CHECK(std::isnan(st.slot_period_s));
    CHECK(std::isnan(st.period_min_s));
    CHECK_FALSE(st.echoes_seen());
}
