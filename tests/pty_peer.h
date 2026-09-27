#pragma once

// Test helper shared by the pty-backed tests.

#include <catch2/catch_test_macros.hpp>

#include <basestation/core/frame_splitter.h>
#include <basestation/core/link_session.h>

#include <poll.h>
#include <termios.h>
#include <unistd.h>
#if defined(__APPLE__)
#include <util.h>
#else
#include <pty.h>
#endif

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace basestation::test {

// A pty pair standing in for the base-station ESP32: LinkSession opens the
// slave, the test reads what it sent from the master.
class PtyPeer {
public:
    struct Received {
        Clock::time_point t;
        wirelink::Frame frame;
    };

    PtyPeer() {
        REQUIRE(::openpty(&master_, &slave_, nullptr, nullptr, nullptr) == 0);
        termios tio{};
        REQUIRE(::tcgetattr(slave_, &tio) == 0);
        ::cfmakeraw(&tio);
        REQUIRE(::tcsetattr(slave_, TCSANOW, &tio) == 0);
        const char* name = ::ttyname(slave_);
        REQUIRE(name != nullptr);
        path_ = name;
    }
    ~PtyPeer() {
        // The slave stays open until here so a closed LinkSession does not
        // turn master reads into EIO.
        ::close(master_);
        ::close(slave_);
    }
    PtyPeer(const PtyPeer&) = delete;
    PtyPeer& operator=(const PtyPeer&) = delete;

    const std::string& path() const { return path_; }

    // Collects frames until `want` have arrived or `timeout` passes.
    std::vector<Received> read(size_t want, std::chrono::milliseconds timeout) {
        std::vector<Received> out;
        const auto deadline = Clock::now() + timeout;
        std::vector<wirelink::Frame> frames;
        uint8_t buf[512];
        while (out.size() < want) {
            const auto left =
                std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
            if (left <= 0) break;
            pollfd pfd{master_, POLLIN, 0};
            if (::poll(&pfd, 1, static_cast<int>(left)) <= 0) continue;
            const ssize_t n = ::read(master_, buf, sizeof buf);
            if (n <= 0) continue;
            const auto now = Clock::now();
            frames.clear();
            splitter_.feed(buf, static_cast<size_t>(n), frames);
            for (const auto& f : frames) out.push_back(Received{now, f});
        }
        return out;
    }
    // Everything that arrives within `window`.
    std::vector<Received> read_for(std::chrono::milliseconds window) { return read(SIZE_MAX, window); }

private:
    int master_ = -1;
    int slave_ = -1;
    std::string path_;
    FrameSplitter splitter_;
};

inline void open_link(LinkSession& link, const PtyPeer& pty) {
    std::string err;
    REQUIRE(link.open(pty.path(), 115200, &err));
}

}  // namespace basestation::test
