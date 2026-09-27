#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <basestation/core/frame_splitter.h>
#include <basestation/core/link_session.h>
#include <basestation/core/teleop.h>
#include <basestation/core/teleop_sender.h>
#include <basestation/proto/codec.h>
#include <basestation/proto/lora.h>

#include <boat_defs/ids.h>

#include "pty_peer.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

using namespace basestation;
using basestation::test::PtyPeer;
using basestation::test::open_link;
using namespace std::chrono_literals;
using Catch::Matchers::WithinAbs;

namespace {

std::vector<lora::Command> commands(const std::vector<PtyPeer::Received>& rx) {
    std::vector<lora::Command> out;
    for (const auto& r : rx) {
        lora::Command c{};
        REQUIRE(codec::unpack(r.frame, c));
        out.push_back(c);
    }
    return out;
}

GamepadSnapshot pad_now(float lin, float ang, bool deadman = true) {
    GamepadSnapshot p;
    p.connected = true;
    p.t = Clock::now();
    p.axes[static_cast<size_t>(GamepadAxis::LeftY)] = lin;
    p.axes[static_cast<size_t>(GamepadAxis::RightX)] = ang;
    p.buttons[static_cast<size_t>(GamepadButton::RightShoulder)] = deadman;
    return p;
}

// Linear shaping so wire values are easy to predict.
TeleopConfig plain_config(float rate_hz = 50.0f) {
    TeleopConfig c;
    c.deadzone = 0.0f;
    c.expo = 0.0f;
    c.max_linear_mps = 1.0f;
    c.max_angular_radps = 2.0f;
    c.rate_hz = rate_hz;
    c.input_stale_ms = 10000;
    c.stop_burst = 3;
    return c;
}

}  // namespace

// ---------------------------------------------------------------------------
// Pure mapping
// ---------------------------------------------------------------------------

TEST_CASE("teleop: names", "[model][teleop]") {
    CHECK(std::string(to_string(GamepadButton::South)) == "A");
    CHECK(std::string(to_string(GamepadButton::East)) == "B");
    CHECK(std::string(to_string(GamepadButton::West)) == "X");
    CHECK(std::string(to_string(GamepadButton::North)) == "Y");
    CHECK(std::string(to_string(GamepadButton::RightShoulder)) == "RB");
    CHECK(std::string(to_string(GamepadButton::Back)) == "View");
    CHECK(std::string(to_string(GamepadButton::DpadUp)) == "D-pad up");
    CHECK(std::string(to_string(GamepadAxis::LeftX)) == "Left stick X");
    CHECK(std::string(to_string(GamepadAxis::RightTrigger)) == "Right trigger");
    for (int i = 0; i < static_cast<int>(GamepadButton::Count); ++i)
        CHECK(std::strlen(to_string(static_cast<GamepadButton>(i))) > 0);
    for (int i = 0; i < static_cast<int>(GamepadAxis::Count); ++i)
        CHECK(std::strlen(to_string(static_cast<GamepadAxis>(i))) > 0);
}

TEST_CASE("teleop: shape_axis deadzone, expo, continuity and sign", "[model][teleop]") {
    const float dz = 0.1f;
    for (float e : {0.0f, 0.3f, 1.0f}) {
        CHECK(shape_axis(0.0f, dz, e) == 0.0f);
        CHECK(shape_axis(0.05f, dz, e) == 0.0f);
        CHECK(shape_axis(-0.1f, dz, e) == 0.0f);
        // No jump just outside the deadzone.
        CHECK(std::fabs(shape_axis(0.1001f, dz, e)) < 1e-3f);
        CHECK_THAT(shape_axis(1.0f, dz, e), WithinAbs(1.0, 1e-6));
        CHECK_THAT(shape_axis(-1.0f, dz, e), WithinAbs(-1.0, 1e-6));
        // Clamped beyond full travel.
        CHECK_THAT(shape_axis(1.5f, dz, e), WithinAbs(1.0, 1e-6));
        CHECK_THAT(shape_axis(-3.0f, dz, e), WithinAbs(-1.0, 1e-6));
        // Odd and monotonic.
        float prev = 0.0f;
        for (int i = 0; i <= 100; ++i) {
            const float x = static_cast<float>(i) / 100.0f;
            const float y = shape_axis(x, dz, e);
            CHECK(y >= prev);
            CHECK(shape_axis(-x, dz, e) == -y);
            prev = y;
        }
    }
    // Formula check: u = (0.55 - 0.1) / 0.9 = 0.5.
    CHECK_THAT(shape_axis(0.55f, dz, 0.0f), WithinAbs(0.5, 1e-6));
    CHECK_THAT(shape_axis(0.55f, dz, 1.0f), WithinAbs(0.125, 1e-6));
    CHECK_THAT(shape_axis(0.55f, dz, 0.3f), WithinAbs(0.7 * 0.5 + 0.3 * 0.125, 1e-6));
    CHECK(shape_axis(NAN, dz, 0.3f) == 0.0f);
    // Triggers only ever give [0, 1].
    CHECK_THAT(shape_axis(0.55f, dz, 0.0f), WithinAbs(0.5, 1e-6));
}

TEST_CASE("teleop: zero command reasons", "[model][teleop]") {
    const TeleopConfig cfg = plain_config();
    const auto now = Clock::now();

    GamepadSnapshot pad = pad_now(1.0f, 1.0f);
    pad.t = now;

    GamepadSnapshot off = pad;
    off.connected = false;
    TeleopOutput o = compute_teleop(off, cfg, now);
    CHECK_FALSE(o.driving);
    CHECK(std::string(o.reason) == "no gamepad");
    CHECK(o.linear_mps == 0.0f);
    CHECK(o.angular_radps == 0.0f);

    TeleopConfig fast_stale = cfg;
    fast_stale.input_stale_ms = 100;
    o = compute_teleop(pad, fast_stale, now + 101ms);
    CHECK_FALSE(o.driving);
    CHECK(std::string(o.reason) == "input stale");
    CHECK(o.linear_mps == 0.0f);
    o = compute_teleop(pad, fast_stale, now + 99ms);
    CHECK(o.driving);

    GamepadSnapshot released = pad;
    released.buttons[static_cast<size_t>(GamepadButton::RightShoulder)] = false;
    o = compute_teleop(released, cfg, now);
    CHECK_FALSE(o.driving);
    CHECK(std::string(o.reason) == "deadman released");
    CHECK(o.angular_radps == 0.0f);

    TeleopConfig no_deadman = cfg;
    no_deadman.require_deadman = false;
    o = compute_teleop(released, no_deadman, now);
    CHECK(o.driving);
    CHECK(o.linear_mps == 1.0f);
}

TEST_CASE("teleop: mapping signs, inversion and scaling", "[model][teleop]") {
    TeleopConfig cfg = plain_config();
    cfg.max_linear_mps = 1.5f;
    cfg.max_angular_radps = 0.8f;
    const auto now = Clock::now();

    GamepadSnapshot pad = pad_now(1.0f, 1.0f);  // stick up, stick right
    pad.t = now;
    TeleopOutput o = compute_teleop(pad, cfg, now);
    CHECK(o.driving);
    CHECK(std::string(o.reason).empty());
    CHECK_THAT(o.linear_mps, WithinAbs(1.5, 1e-6));     // up -> forward
    CHECK_THAT(o.angular_radps, WithinAbs(-0.8, 1e-6));  // right -> clockwise

    pad.axes[static_cast<size_t>(GamepadAxis::LeftY)] = -0.5f;
    pad.axes[static_cast<size_t>(GamepadAxis::RightX)] = -0.25f;
    o = compute_teleop(pad, cfg, now);
    CHECK_THAT(o.linear_mps, WithinAbs(-0.75, 1e-6));
    CHECK_THAT(o.angular_radps, WithinAbs(0.2, 1e-6));

    cfg.invert_linear = true;
    cfg.invert_angular = true;
    o = compute_teleop(pad, cfg, now);
    CHECK_THAT(o.linear_mps, WithinAbs(0.75, 1e-6));
    CHECK_THAT(o.angular_radps, WithinAbs(-0.2, 1e-6));

    // Other axes / a trigger as throttle.
    cfg = plain_config();
    cfg.linear_axis = GamepadAxis::RightTrigger;
    cfg.angular_axis = GamepadAxis::LeftX;
    pad.axes[static_cast<size_t>(GamepadAxis::RightTrigger)] = 0.5f;
    pad.axes[static_cast<size_t>(GamepadAxis::LeftX)] = 0.0f;
    o = compute_teleop(pad, cfg, now);
    CHECK_THAT(o.linear_mps, WithinAbs(0.5, 1e-6));
    CHECK(o.angular_radps == 0.0f);
    CHECK_FALSE(std::signbit(o.angular_radps));
}

// ---------------------------------------------------------------------------
// TeleopSender over a pty
// ---------------------------------------------------------------------------

TEST_CASE("TeleopSender: sends commands at the configured rate", "[model][teleop][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    EventLog log;
    TeleopSender sender(link, &log);
    sender.set_config(plain_config(20.0f));
    sender.set_target(3);
    sender.update_input(pad_now(0.5f, 0.25f));

    CHECK(pty.read_for(60ms).empty());  // nothing while disabled

    sender.set_enabled(true);
    const auto rx = pty.read_for(260ms);
    // 20 Hz for ~260 ms: ~6 frames. Loose bounds for loaded CI machines.
    CHECK(rx.size() >= 3);
    CHECK(rx.size() <= 9);
    for (const auto& c : commands(rx)) {
        CHECK(c.rx_id == 3);
        CHECK(c.lin_vel == 500);   // 0.5 m/s
        CHECK(c.ang_vel == -500);  // stick right -> -0.5 rad/s
    }
    const auto st = sender.status();
    CHECK(st.enabled);
    CHECK(st.target == 3);
    CHECK(st.last.driving);
    CHECK(st.sent >= rx.size());
    CHECK(st.send_failures == 0);

    CHECK(log.snapshot().size() >= 2);  // target + enabled
}

TEST_CASE("TeleopSender: disable sends the stop burst then goes quiet", "[model][teleop][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    TeleopSender sender(link);
    sender.set_config(plain_config(50.0f));
    sender.set_target(2);
    sender.update_input(pad_now(1.0f, 0.0f));
    sender.set_enabled(true);
    REQUIRE(pty.read(3, 500ms).size() == 3);

    sender.set_enabled(false);
    auto rx = commands(pty.read_for(150ms));
    // At most one driving command can have been in flight when disabled.
    REQUIRE(rx.size() >= 3);
    REQUIRE(rx.size() <= 5);
    const size_t zeros_from = rx.size() - 3;
    for (size_t i = 0; i < rx.size(); ++i) {
        CHECK(rx[i].rx_id == 2);
        if (i >= zeros_from) {
            CHECK(rx[i].lin_vel == 0);
            CHECK(rx[i].ang_vel == 0);
        }
    }
    CHECK(pty.read_for(60ms).empty());
    CHECK_FALSE(sender.status().enabled);
}

TEST_CASE("TeleopSender: stale input sends zero", "[model][teleop][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    TeleopSender sender(link);
    TeleopConfig cfg = plain_config(50.0f);
    cfg.input_stale_ms = 50;
    sender.set_config(cfg);
    sender.set_target(1);
    GamepadSnapshot pad = pad_now(1.0f, 1.0f);
    pad.t = Clock::now() - 1s;
    sender.update_input(pad);
    sender.set_enabled(true);

    const auto rx = commands(pty.read(3, 500ms));
    REQUIRE(rx.size() == 3);
    for (const auto& c : rx) {
        CHECK(c.rx_id == 1);
        CHECK(c.lin_vel == 0);
        CHECK(c.ang_vel == 0);
    }
    const auto st = sender.status();
    CHECK_FALSE(st.last.driving);
    CHECK(std::string(st.last.reason) == "input stale");
}

TEST_CASE("TeleopSender: target change stops the old target first", "[model][teleop][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    TeleopSender sender(link);
    sender.set_config(plain_config(50.0f));
    sender.set_target(1);
    sender.update_input(pad_now(0.2f, 0.0f));
    sender.set_enabled(true);
    REQUIRE(pty.read(2, 500ms).size() == 2);

    sender.set_target(4);
    auto rx = commands(pty.read(10, 600ms));
    REQUIRE(rx.size() == 10);
    // Driving commands to boat 1 may still be in flight, then 3 zeros to
    // boat 1, then boat 4 only.
    size_t i = 0;
    while (i < rx.size() && rx[i].lin_vel != 0) {
        CHECK(rx[i].rx_id == 1);
        ++i;
    }
    CHECK(i <= 2);
    REQUIRE(i + 3 < rx.size());
    for (size_t k = 0; k < 3; ++k, ++i) {
        CHECK(rx[i].rx_id == 1);
        CHECK(rx[i].lin_vel == 0);
    }
    for (; i < rx.size(); ++i) {
        CHECK(rx[i].rx_id == 4);
        CHECK(rx[i].lin_vel == 200);
    }
    CHECK(sender.status().target == 4);
}

TEST_CASE("TeleopSender: destructor sends the stop burst", "[model][teleop][pty]") {
    PtyPeer pty;
    LinkSession link;
    open_link(link, pty);
    {
        TeleopSender sender(link);
        TeleopConfig cfg = plain_config(5.0f);  // slow, so the burst must come from the destructor
        cfg.stop_burst = 2;
        sender.set_config(cfg);
        sender.set_target(boat::ids::BROADCAST_ID);
        sender.update_input(pad_now(1.0f, 0.0f));
        sender.set_enabled(true);
        REQUIRE(pty.read(1, 500ms).size() == 1);
    }
    const auto rx = commands(pty.read_for(150ms));
    REQUIRE(rx.size() >= 2);
    CHECK(rx.back().rx_id == boat::ids::BROADCAST_ID);
    CHECK(rx.back().lin_vel == 0);
    CHECK(rx[rx.size() - 2].lin_vel == 0);
}

TEST_CASE("TeleopSender: closed link counts failures and warns once", "[model][teleop][pty]") {
    LinkSession link;  // never opened
    EventLog log;
    TeleopSender sender(link, &log);
    sender.set_config(plain_config(50.0f));
    sender.update_input(pad_now(0.0f, 0.0f));
    sender.set_enabled(true);
    std::this_thread::sleep_for(120ms);
    const auto st = sender.status();
    CHECK(st.sent == 0);
    CHECK(st.send_failures >= 2);
    size_t warns = 0;
    for (const auto& e : log.snapshot())
        if (e.level == EventLevel::Warn) ++warns;
    CHECK(warns == 1);
}
