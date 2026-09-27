#include <basestation/core/link_session.h>
#include <basestation/proto/lora.h>

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#ifdef __APPLE__
#include <util.h>
#else
#include <pty.h>
#endif

using namespace basestation;
using namespace std::chrono_literals;

namespace {

// A pty pair standing in for the USB serial port: the session opens the
// slave by path, the test plays the ESP32 on the master.
struct FakePort {
    int master = -1;
    int slave = -1;
    std::string path;

    FakePort() {
        REQUIRE(::openpty(&master, &slave, nullptr, nullptr, nullptr) == 0);
        termios tio{};
        REQUIRE(::tcgetattr(slave, &tio) == 0);
        ::cfmakeraw(&tio);
        REQUIRE(::tcsetattr(slave, TCSANOW, &tio) == 0);
        path = ::ttyname(slave);
        ::fcntl(master, F_SETFL, ::fcntl(master, F_GETFL) | O_NONBLOCK);
    }
    ~FakePort() {
        close_master();
        if (slave >= 0) ::close(slave);
    }
    void close_master() {
        if (master >= 0) ::close(master);
        master = -1;
    }

    void write_frame(const wirelink::Frame& f, uint8_t seq) {
        const auto p = wirelink::framing::wrap(f, seq);
        REQUIRE(p);
        REQUIRE(::write(master, p->data.data(), p->len) == static_cast<ssize_t>(p->len));
    }

    // Reads frames the session sent, until `count` arrive or 2 s pass.
    std::vector<wirelink::Frame> read_frames(size_t count) {
        std::vector<wirelink::Frame> frames;
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (frames.size() < count && std::chrono::steady_clock::now() < deadline) {
            pollfd pfd{master, POLLIN, 0};
            if (::poll(&pfd, 1, 20) <= 0) continue;
            uint8_t buf[512];
            const ssize_t n = ::read(master, buf, sizeof buf);
            if (n > 0) splitter.feed(buf, static_cast<size_t>(n), frames);
        }
        return frames;
    }

    FrameSplitter splitter;
};

template <class Pred>
bool wait_for(Pred pred) {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!pred()) {
        if (std::chrono::steady_clock::now() > deadline) return false;
        std::this_thread::sleep_for(5ms);
    }
    return true;
}

wirelink::Frame status_frame(uint8_t tx_id) {
    wirelink::Frame f{};
    REQUIRE(codec::pack(lora::Status{tx_id, 1, 1, 0, 0, 90.0f, -80.0f, 5.0f}, f));
    return f;
}

bool has_event(const EventLog& log, EventLevel level) {
    for (const auto& e : log.snapshot()) {
        if (e.level == level) return true;
    }
    return false;
}

}  // namespace

TEST_CASE("link session receives frames", "[io]") {
    FakePort port;
    EventLog events;
    LinkSession session(&events);
    std::string err;
    REQUIRE(session.open(port.path, 115200, &err));
    CHECK(session.is_open());
    CHECK(session.path() == port.path);
    CHECK(session.baud() == 115200);
    CHECK(has_event(events, EventLevel::Info));

    const auto before = Clock::now();
    for (uint8_t i = 0; i < 5; ++i) port.write_frame(status_frame(i), static_cast<uint8_t>(10 + i));

    std::vector<RxFrame> rx;
    REQUIRE(wait_for([&] { session.poll(rx); return rx.size() >= 5; }));
    REQUIRE(rx.size() == 5);
    for (uint8_t i = 0; i < 5; ++i) {
        CHECK(rx[i].t >= before);
        CHECK(rx[i].frame.seq == 10 + i);
        lora::Status s{};
        REQUIRE(codec::unpack(rx[i].frame, s));
        CHECK(s.tx_id == i);
    }
    std::vector<RxFrame> more;
    CHECK(session.poll(more) == 0);

    const auto st = session.stats();
    CHECK(st.rx_frames == 5);
    CHECK(st.rx_bad == 0);
    CHECK(st.rx_bytes > 0);

    session.close();
    CHECK_FALSE(session.is_open());
    CHECK(session.last_error().empty());
    session.close();  // idempotent
}

TEST_CASE("link session counts corrupt frames", "[io]") {
    FakePort port;
    LinkSession session;
    REQUIRE(session.open(port.path, 115200, nullptr));

    const auto p = wirelink::framing::wrap(status_frame(1), 0);
    REQUIRE(p);
    std::vector<uint8_t> bad(p->data.begin(), p->data.begin() + p->len);
    bad[5] ^= 0x5A;
    REQUIRE(bad[5] != 0);
    REQUIRE(::write(port.master, bad.data(), bad.size()) == static_cast<ssize_t>(bad.size()));
    port.write_frame(status_frame(2), 1);

    std::vector<RxFrame> rx;
    REQUIRE(wait_for([&] { session.poll(rx); return !rx.empty(); }));
    CHECK(rx.size() == 1);
    CHECK(session.stats().rx_bad == 1);
}

TEST_CASE("link session sends frames with per-type sequence numbers", "[io]") {
    FakePort port;
    LinkSession session;
    REQUIRE(session.open(port.path, 115200, nullptr));

    CHECK(session.send_msg(lora::Command{1, 100, -200}));
    CHECK(session.send_msg(lora::Command{1, 101, -201}));
    CHECK(session.send_msg(lora::Disable{0xFF}));
    CHECK(session.send_msg(lora::Command{2, 102, -202}));

    const auto frames = port.read_frames(4);
    REQUIRE(frames.size() == 4);
    CHECK(port.splitter.stats().frames_bad == 0);

    lora::Command c{};
    REQUIRE(codec::unpack(frames[0], c));
    CHECK(frames[0].seq == 0);
    CHECK(c.lin_vel == 100);
    REQUIRE(codec::unpack(frames[1], c));
    CHECK(frames[1].seq == 1);
    CHECK(c.ang_vel == -201);
    lora::Disable d{};
    REQUIRE(codec::unpack(frames[2], d));
    CHECK(frames[2].seq == 0);  // Disable has its own counter
    CHECK(d.rx_id == 0xFF);
    REQUIRE(codec::unpack(frames[3], c));
    CHECK(frames[3].seq == 2);
    CHECK(c.rx_id == 2);

    const auto st = session.stats();
    CHECK(st.tx_frames == 4);
    CHECK(st.tx_bytes > 4 * 8);
    CHECK(st.tx_errors == 0);

    session.close();
    CHECK_FALSE(session.send_msg(lora::Disable{1}));
}

TEST_CASE("link session records rx and tx to the frame log", "[io]") {
    const auto dir = std::filesystem::temp_directory_path() /
                     ("asep-link-session-test-" + std::to_string(::getpid()));
    const std::string path = (dir / "session.aseplog").string();
    {
        FakePort port;
        FrameLogWriter log;
        REQUIRE(log.open(path, nullptr));
        LinkSession session(nullptr, &log);
        REQUIRE(session.open(port.path, 115200, nullptr));

        port.write_frame(status_frame(4), 77);
        std::vector<RxFrame> rx;
        REQUIRE(wait_for([&] { session.poll(rx); return !rx.empty(); }));
        REQUIRE(session.send_msg(lora::SetMode{4, 2, 1}));
        REQUIRE(session.send_msg(lora::SetMode{4, 3, 1}));
        CHECK(port.read_frames(2).size() == 2);
        session.close();
        CHECK(log.records() == 3);
    }

    FrameLogReader reader;
    REQUIRE(reader.open(path, nullptr));
    LogRecord rec;
    REQUIRE(reader.next(rec));
    CHECK(rec.dir == Direction::Rx);
    CHECK(rec.frame.seq == 77);
    lora::Status s{};
    CHECK(codec::unpack(rec.frame, s));
    for (uint8_t seq = 0; seq < 2; ++seq) {
        REQUIRE(reader.next(rec));
        CHECK(rec.dir == Direction::Tx);
        CHECK(rec.frame.seq == seq);  // the seq actually sent
        lora::SetMode m{};
        CHECK(codec::unpack(rec.frame, m));
    }
    CHECK_FALSE(reader.next(rec));
    reader.close();
    std::filesystem::remove_all(dir);
}

TEST_CASE("link session closes itself when the port goes away", "[io]") {
    FakePort port;
    EventLog events;
    LinkSession session(&events);
    REQUIRE(session.open(port.path, 115200, nullptr));

    // Closing the master hangs up the slave, like unplugging the USB cable.
    port.close_master();
    REQUIRE(wait_for([&] { return !session.is_open(); }));
    CHECK_FALSE(session.last_error().empty());
    CHECK(has_event(events, EventLevel::Error));
    CHECK_FALSE(session.send_msg(lora::Disable{1}));

    // close() after a self-close still reaps the reader thread cleanly.
    session.close();
    CHECK_FALSE(session.is_open());
}

TEST_CASE("link session open reports errors", "[io]") {
    LinkSession session;
    std::string err;
    CHECK_FALSE(session.open("/dev/does-not-exist-asep", 115200, &err));
    CHECK_FALSE(err.empty());
    CHECK_FALSE(session.is_open());
}

TEST_CASE("serial port helpers", "[io]") {
    CHECK(common_baud_rates().size() == 8);
    CHECK(common_baud_rates().front() == 9600);
    CHECK(common_baud_rates().back() == 921600);

    const auto ports = list_serial_ports();  // usually empty in CI; must not throw
    for (size_t i = 1; i < ports.size(); ++i) CHECK(ports[i - 1].path < ports[i].path);

#ifdef __linux__
    FakePort port;
    SerialPort sp;
    std::string err;
    CHECK_FALSE(sp.open(port.path, 123456, &err));  // no custom rates on Linux
    CHECK_FALSE(err.empty());
#endif
}
