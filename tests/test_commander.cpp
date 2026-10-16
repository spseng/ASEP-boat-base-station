#include <catch2/catch_test_macros.hpp>

#include <basestation/core/commander.h>
#include <basestation/core/uplink.h>
#include <basestation/core/frame_splitter.h>
#include <basestation/core/link_session.h>
#include <basestation/proto/msgs.h>

#include <boat_defs/ids.h>
#include <boat_defs/mode.h>

#include <wirelink/wire.h>

#include "pty_peer.h"

#include <cstdint>
#include <string>
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

// Echoes come from the test (Uplink::on_frame), never from a timeout.
Uplink::Config no_timeouts() {
    Uplink::Config c;
    c.echo_timeout = 60s;
    c.max_age = 60s;
    return c;
}

// The ESP32 transmitted `f` in its land slot.
void echo(Uplink& up, const wirelink::Frame& f) { up.on_frame(RxFrame{Clock::now(), f}); }

template <class T>
bool is(const wirelink::Frame& f, uint8_t rx_id) {
    T m{};
    return f.type == static_cast<uint8_t>(T::TYPE) && wirelink::codec::unpack(f, m) && m.rx_id == rx_id;
}

// Exactly one frame handed over.
wirelink::Frame one(PtyPeer& pty) {
    const auto rx = pty.read(1, 1000ms);
    REQUIRE(rx.size() == 1);
    REQUIRE(pty.read_for(30ms).empty());
    return rx[0].frame;
}

}  // namespace

TEST_CASE("Commander: target_name", "[model][commander]") {
    CHECK(Commander::target_name(3) == "boat 3");
    CHECK(Commander::target_name(boat::ids::BROADCAST_ID) == "ALL");
}

TEST_CASE("Commander: disable goes out in disable_repeats land slots", "[model][commander][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    Uplink up(link);
    up.set_config(no_timeouts());
    EventLog log;
    Commander cmd(up, &log);
    cmd.set_config(Commander::Config{3});
    CHECK(cmd.config().disable_repeats == 3);

    REQUIRE(cmd.disable(3));
    for (int copy = 0; copy < 3; ++copy) {
        const wirelink::Frame f = one(pty);
        CHECK(is<lora::Disable>(f, 3));
        echo(up, f);
    }
    CHECK(pty.read_for(30ms).empty());
    CHECK(up.status().queued == 0);
    CHECK(count_events(log, EventLevel::Warn, "Sent DISABLE to boat 3") == 1);
}

TEST_CASE("Commander: disable to ALL", "[model][commander][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    Uplink up(link);
    up.set_config(no_timeouts());
    EventLog log;
    Commander cmd(up, &log);
    cmd.set_config(Commander::Config{1});
    REQUIRE(cmd.disable(boat::ids::BROADCAST_ID));
    const wirelink::Frame f = one(pty);
    CHECK(is<lora::Disable>(f, boat::ids::BROADCAST_ID));
    echo(up, f);
    CHECK(pty.read_for(30ms).empty());
    CHECK(count_events(log, EventLevel::Warn, "Sent DISABLE to ALL") == 1);
}

TEST_CASE("Commander: reenable and set_mode go out once, in order", "[model][commander][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    Uplink up(link);
    up.set_config(no_timeouts());
    EventLog log;
    Commander cmd(up, &log);

    REQUIRE(cmd.reenable(2));
    REQUIRE(cmd.set_mode(1, boat::mode::Mode::AUTONOMOUS, boat::mode::ArmedState::ARMED));
    const wirelink::Frame r = one(pty);  // the SetMode waits for the Reenable's slot
    CHECK(is<lora::Reenable>(r, 2));
    CHECK(count_events(log, EventLevel::Info, "Sent REENABLE to boat 2") == 1);

    echo(up, r);
    const wirelink::Frame m = one(pty);
    lora::SetMode sm{};
    REQUIRE(wirelink::codec::unpack(m, sm));
    CHECK(sm.rx_id == 1);
    CHECK(sm.mode == static_cast<uint8_t>(boat::mode::Mode::AUTONOMOUS));
    CHECK(sm.armed == static_cast<uint8_t>(boat::mode::ArmedState::ARMED));
    CHECK(count_events(log, EventLevel::Info, "Sent SET_MODE AUTONOMOUS ARMED to boat 1") == 1);

    echo(up, m);
    CHECK(pty.read_for(30ms).empty());  // neither is repeated
    CHECK(up.status().queued == 0);
}

TEST_CASE("Commander: reenable cancels disable copies still owed", "[model][commander][pty]") {
    using boat::ids::BROADCAST_ID;
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    Uplink up(link);
    up.set_config(no_timeouts());
    Commander cmd(up);
    cmd.set_config(Commander::Config{5});

    SECTION("same boat") {
        REQUIRE(cmd.disable(3));
        const wirelink::Frame d = one(pty);
        echo(up, d);
        CHECK(is<lora::Disable>(one(pty), 3));  // copy 2 in the mailbox
        REQUIRE(cmd.reenable(3));
        const wirelink::Frame r = one(pty);  // replaces it at once
        CHECK(is<lora::Reenable>(r, 3));
        echo(up, r);
        CHECK(pty.read_for(30ms).empty());
        CHECK(up.status().queued == 0);
    }
    SECTION("reenable ALL drops every disable") {
        REQUIRE(cmd.disable(2));
        CHECK(is<lora::Disable>(one(pty), 2));
        REQUIRE(cmd.disable(BROADCAST_ID));  // queued behind the first
        CHECK(pty.read_for(30ms).empty());
        REQUIRE(cmd.reenable(BROADCAST_ID));
        const wirelink::Frame r = one(pty);
        CHECK(is<lora::Reenable>(r, BROADCAST_ID));
        echo(up, r);
        CHECK(pty.read_for(30ms).empty());
        CHECK(up.status().queued == 0);
    }
    SECTION("reenabling one boat keeps a broadcast disable") {
        REQUIRE(cmd.disable(BROADCAST_ID));
        const wirelink::Frame all = one(pty);
        REQUIRE(cmd.disable(2));
        REQUIRE(cmd.reenable(2));
        CHECK(pty.read_for(30ms).empty());  // Disable(ALL) stays in the mailbox
        for (int copy = 0; copy < 5; ++copy) {
            echo(up, all);
            const wirelink::Frame next = one(pty);
            if (copy < 4) {
                CHECK(is<lora::Disable>(next, BROADCAST_ID));
            } else {
                CHECK(is<lora::Reenable>(next, 2));  // never a Disable to boat 2
                echo(up, next);
            }
        }
        CHECK(pty.read_for(30ms).empty());
        CHECK(up.status().queued == 0);
    }
}

TEST_CASE("Commander: closed link reports failure", "[model][commander]") {
    LinkSession link;  // never opened
    Uplink up(link);
    EventLog log;
    Commander cmd(up, &log);
    CHECK_FALSE(cmd.disable(3));
    CHECK_FALSE(cmd.reenable(3));
    CHECK_FALSE(cmd.set_mode(3, boat::mode::Mode::MANUAL, boat::mode::ArmedState::DISARMED));
    CHECK(count_events(log, EventLevel::Error, "DISABLE") == 1);
    CHECK(count_events(log, EventLevel::Error, "REENABLE") == 1);
    CHECK(count_events(log, EventLevel::Error, "SET_MODE") == 1);
}
