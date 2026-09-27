#include <catch2/catch_test_macros.hpp>

#include <basestation/core/commander.h>
#include <basestation/core/frame_splitter.h>
#include <basestation/core/link_session.h>
#include <basestation/proto/codec.h>
#include <basestation/proto/lora.h>

#include <boat_defs/ids.h>
#include <boat_defs/mode.h>

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

}  // namespace

TEST_CASE("Commander: target_name", "[model][commander]") {
    CHECK(Commander::target_name(3) == "boat 3");
    CHECK(Commander::target_name(boat::ids::BROADCAST_ID) == "ALL");
}

TEST_CASE("Commander: disable is sent at once and repeated", "[model][commander][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    EventLog log;
    Commander cmd(link, &log);
    cmd.set_config(Commander::Config{3, 60ms});
    CHECK(cmd.config().disable_repeats == 3);

    const auto t0 = Clock::now();
    REQUIRE(cmd.disable(3));
    const auto rx = pty.read(3, 400ms);
    REQUIRE(rx.size() == 3);
    CHECK(pty.read_for(90ms).empty());  // exactly disable_repeats copies
    for (size_t i = 0; i < rx.size(); ++i) {
        lora::Disable d{};
        REQUIRE(codec::unpack(rx[i].frame, d));
        CHECK(d.rx_id == 3);
        CHECK(rx[i].frame.seq == i);  // LinkSession numbers each copy
    }
    // First copy immediately, the rest roughly repeat_interval apart.
    CHECK(rx[0].t - t0 < 50ms);
    CHECK(rx[1].t - rx[0].t >= 40ms);
    CHECK(rx[2].t - rx[1].t >= 40ms);
    CHECK(rx[2].t - rx[0].t < 300ms);
    CHECK(count_events(log, EventLevel::Warn, "Sent DISABLE to boat 3") == 1);
}

TEST_CASE("Commander: disable to ALL", "[model][commander][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    EventLog log;
    Commander cmd(link, &log);
    cmd.set_config(Commander::Config{1, 10ms});
    REQUIRE(cmd.disable(boat::ids::BROADCAST_ID));
    const auto rx = pty.read_for(60ms);
    REQUIRE(rx.size() == 1);
    lora::Disable d{};
    REQUIRE(codec::unpack(rx[0].frame, d));
    CHECK(d.rx_id == boat::ids::BROADCAST_ID);
    CHECK(count_events(log, EventLevel::Warn, "Sent DISABLE to ALL") == 1);
}

TEST_CASE("Commander: reenable and set_mode are sent once", "[model][commander][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    EventLog log;
    Commander cmd(link, &log);

    REQUIRE(cmd.reenable(2));
    auto rx = pty.read(1, 200ms);
    REQUIRE(rx.size() == 1);
    lora::Reenable r{};
    REQUIRE(codec::unpack(rx[0].frame, r));
    CHECK(r.rx_id == 2);
    CHECK(count_events(log, EventLevel::Info, "Sent REENABLE to boat 2") == 1);

    REQUIRE(cmd.set_mode(1, boat::mode::Mode::AUTONOMOUS, boat::mode::ArmedState::ARMED));
    rx = pty.read(1, 200ms);
    REQUIRE(rx.size() == 1);
    lora::SetMode m{};
    REQUIRE(codec::unpack(rx[0].frame, m));
    CHECK(m.rx_id == 1);
    CHECK(m.mode == static_cast<uint8_t>(boat::mode::Mode::AUTONOMOUS));
    CHECK(m.armed == static_cast<uint8_t>(boat::mode::ArmedState::ARMED));
    CHECK(count_events(log, EventLevel::Info, "Sent SET_MODE AUTONOMOUS ARMED to boat 1") == 1);
    CHECK(pty.read_for(60ms).empty());  // neither is repeated
}

template <class T>
bool is(const PtyPeer::Received& r, uint8_t rx_id) {
    T m{};
    return codec::unpack(r.frame, m) && m.rx_id == rx_id;
}

TEST_CASE("Commander: reenable cancels queued disable repeats", "[model][commander][pty]") {
    using boat::ids::BROADCAST_ID;
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    Commander cmd(link);
    cmd.set_config(Commander::Config{5, 40ms});

    SECTION("same boat") {
        REQUIRE(cmd.disable(3));
        REQUIRE(pty.read(1, 200ms).size() == 1);
        REQUIRE(cmd.reenable(3));
        const auto rx = pty.read_for(120ms);
        // A repeat that went out before the reenable is fine; nothing after it.
        REQUIRE(!rx.empty());
        CHECK(is<lora::Reenable>(rx.back(), 3));
        for (size_t i = 0; i + 1 < rx.size(); ++i) CHECK(is<lora::Disable>(rx[i], 3));
        CHECK(rx.size() <= 2);
    }
    SECTION("reenable ALL drops every pending disable") {
        REQUIRE(cmd.disable(2));
        REQUIRE(cmd.disable(BROADCAST_ID));
        REQUIRE(pty.read(2, 200ms).size() == 2);
        REQUIRE(cmd.reenable(BROADCAST_ID));
        const auto rx = pty.read_for(120ms);
        REQUIRE(!rx.empty());
        CHECK(is<lora::Reenable>(rx.back(), BROADCAST_ID));
        CHECK(rx.size() <= 3);
    }
    SECTION("reenabling one boat keeps a broadcast disable repeating") {
        REQUIRE(cmd.disable(BROADCAST_ID));
        REQUIRE(cmd.disable(2));
        REQUIRE(pty.read(2, 200ms).size() == 2);
        REQUIRE(cmd.reenable(2));
        const auto rx = pty.read_for(120ms);
        size_t broadcast = 0;
        size_t to_2_after_reenable = 0;
        bool reenabled = false;
        for (const auto& r : rx) {
            if (is<lora::Reenable>(r, 2)) reenabled = true;
            if (is<lora::Disable>(r, BROADCAST_ID)) ++broadcast;
            if (reenabled && is<lora::Disable>(r, 2)) ++to_2_after_reenable;
        }
        CHECK(reenabled);
        CHECK(broadcast >= 2);
        CHECK(to_2_after_reenable == 0);
    }
}

TEST_CASE("Commander: destructor drops pending repeats", "[model][commander][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    {
        Commander cmd(link);
        cmd.set_config(Commander::Config{5, 200ms});
        REQUIRE(cmd.disable(4));
    }
    CHECK(pty.read_for(60ms).size() == 1);
}

TEST_CASE("Commander: closed link reports failure", "[model][commander]") {
    LinkSession link;  // never opened
    EventLog log;
    Commander cmd(link, &log);
    cmd.set_config(Commander::Config{2, 10ms});
    CHECK_FALSE(cmd.disable(5));
    CHECK_FALSE(cmd.reenable(5));
    CHECK_FALSE(cmd.set_mode(5, boat::mode::Mode::MANUAL, boat::mode::ArmedState::DISARMED));
    CHECK(count_events(log, EventLevel::Error, "DISABLE") == 1);
    CHECK(count_events(log, EventLevel::Error, "REENABLE") == 1);
    CHECK(count_events(log, EventLevel::Error, "SET_MODE") == 1);
}
