#include <basestation/core/serial_port.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>

#include <fcntl.h>
#include <glob.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#endif

#ifdef __linux__
#include <climits>
#include <cstdlib>
#endif

namespace basestation {

namespace {

std::string errno_text(const char* what) {
    return std::string(what) + ": " + std::strerror(errno);
}

void set_error(std::string* error, std::string text) {
    if (error) *error = std::move(text);
}

// Standard termios speed constant for `baud`, or false if there is none.
bool standard_speed(uint32_t baud, speed_t& out) {
    switch (baud) {
#ifdef B1200
        case 1200: out = B1200; return true;
#endif
#ifdef B2400
        case 2400: out = B2400; return true;
#endif
#ifdef B4800
        case 4800: out = B4800; return true;
#endif
#ifdef B9600
        case 9600: out = B9600; return true;
#endif
#ifdef B19200
        case 19200: out = B19200; return true;
#endif
#ifdef B38400
        case 38400: out = B38400; return true;
#endif
#ifdef B57600
        case 57600: out = B57600; return true;
#endif
#ifdef B115200
        case 115200: out = B115200; return true;
#endif
#ifdef B230400
        case 230400: out = B230400; return true;
#endif
#ifdef B460800
        case 460800: out = B460800; return true;
#endif
#ifdef B500000
        case 500000: out = B500000; return true;
#endif
#ifdef B576000
        case 576000: out = B576000; return true;
#endif
#ifdef B921600
        case 921600: out = B921600; return true;
#endif
#ifdef B1000000
        case 1000000: out = B1000000; return true;
#endif
#ifdef B1500000
        case 1500000: out = B1500000; return true;
#endif
#ifdef B2000000
        case 2000000: out = B2000000; return true;
#endif
#ifdef B3000000
        case 3000000: out = B3000000; return true;
#endif
        default: return false;
    }
}

std::string basename_of(const std::string& path) {
    const size_t slash = path.find_last_of('/');
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

std::vector<std::string> glob_paths(const char* pattern) {
    std::vector<std::string> out;
    glob_t g{};
    if (::glob(pattern, 0, nullptr, &g) == 0) {
        for (size_t i = 0; i < g.gl_pathc; ++i) out.emplace_back(g.gl_pathv[i]);
    }
    ::globfree(&g);
    return out;
}

int remaining_ms(std::chrono::steady_clock::time_point deadline) {
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count();
    return left > 0 ? static_cast<int>(left) : 0;
}

}  // namespace

std::vector<SerialPortInfo> list_serial_ports() {
    std::vector<SerialPortInfo> ports;
#ifdef __linux__
    // by-id names identify the adapter ("usb-Silicon_Labs_CP2102_...");
    // attach them to the ttyUSB/ttyACM node they point at.
    std::vector<std::pair<std::string, std::string>> by_id;  // (real path, by-id name)
    for (const auto& link : glob_paths("/dev/serial/by-id/*")) {
        char real[PATH_MAX];
        const std::string target = ::realpath(link.c_str(), real) ? std::string(real) : std::string();
        by_id.emplace_back(target, basename_of(link));
        ports.push_back({link, target.empty() ? std::string() : basename_of(target)});
    }
    for (const char* pattern : {"/dev/ttyUSB*", "/dev/ttyACM*"}) {
        for (const auto& path : glob_paths(pattern)) {
            std::string description;
            for (const auto& [target, name] : by_id) {
                if (target == path) { description = name; break; }
            }
            ports.push_back({path, description});
        }
    }
#elif defined(__APPLE__)
    for (const auto& path : glob_paths("/dev/cu.*")) {
        const std::string name = basename_of(path);
        if (name == "cu.Bluetooth-Incoming-Port") continue;
        ports.push_back({path, name.substr(3)});
    }
#endif
    std::sort(ports.begin(), ports.end(),
              [](const SerialPortInfo& a, const SerialPortInfo& b) { return a.path < b.path; });
    ports.erase(std::unique(ports.begin(), ports.end(),
                            [](const SerialPortInfo& a, const SerialPortInfo& b) { return a.path == b.path; }),
                ports.end());
    return ports;
}

const std::vector<uint32_t>& common_baud_rates() {
    static const std::vector<uint32_t> rates = {
        9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600,
    };
    return rates;
}

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& path, uint32_t baud, std::string* error) {
    close();

    speed_t speed = 0;
    const bool standard = standard_speed(baud, speed);
#ifndef __APPLE__
    if (!standard) {
        set_error(error, "Unsupported baud rate " + std::to_string(baud) + " on this platform");
        return false;
    }
#endif

    const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        set_error(error, errno_text(("Cannot open " + path).c_str()));
        return false;
    }
    auto fail = [&](const char* what) {
        set_error(error, errno_text((path + ": " + what).c_str()));
        ::close(fd);
        return false;
    };

    // Keep other programs (a second instance, a stray terminal) from opening
    // the same port and stealing bytes. Not supported everywhere; best effort.
#ifdef TIOCEXCL
    (void)::ioctl(fd, TIOCEXCL);
#endif

    termios tio{};
    if (::tcgetattr(fd, &tio) != 0) return fail("tcgetattr");
    ::cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~static_cast<tcflag_t>(CSTOPB | PARENB | CSIZE);
    tio.c_cflag |= CS8;
#ifdef CRTSCTS
    tio.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);
#endif
    tio.c_iflag &= ~static_cast<tcflag_t>(IXON | IXOFF | IXANY);
    // Non-blocking reads; waiting is done with poll().
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (standard) {
        if (::cfsetispeed(&tio, speed) != 0 || ::cfsetospeed(&tio, speed) != 0) return fail("cfsetspeed");
    }
    if (::tcsetattr(fd, TCSANOW, &tio) != 0) return fail("tcsetattr");

#ifdef __APPLE__
    if (!standard) {
        // macOS accepts any rate the driver supports, but only through this
        // ioctl, and only after tcsetattr (which would otherwise reset it).
        speed_t custom = static_cast<speed_t>(baud);
        if (::ioctl(fd, IOSSIOSPEED, &custom) != 0) return fail("IOSSIOSPEED");
    }
#endif

    // Drop whatever the device sent before we were listening (boot banner,
    // half a frame). Harmless if unsupported, e.g. on some ptys.
    (void)::tcflush(fd, TCIOFLUSH);

    fd_ = fd;
    path_ = path;
    baud_ = baud;
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

long SerialPort::read(uint8_t* buf, size_t cap, int timeout_ms, std::string* error) {
    if (fd_ < 0) {
        set_error(error, "Port is not open");
        return -1;
    }
    pollfd pfd{fd_, POLLIN, 0};
    const int rc = ::poll(&pfd, 1, timeout_ms);
    if (rc < 0) {
        if (errno == EINTR) return 0;
        set_error(error, errno_text("poll"));
        return -1;
    }
    if (rc == 0) return 0;

    if (pfd.revents & POLLNVAL) {
        set_error(error, "Port descriptor is no longer valid");
        return -1;
    }
    // Read before honouring a hangup so bytes that arrived just before it
    // are not lost; the next call reports the hangup.
    if (pfd.revents & POLLIN) {
        const ssize_t n = ::read(fd_, buf, cap);
        if (n > 0) return static_cast<long>(n);
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return 0;
        // Readable but nothing to read is how a tty reports a hangup.
        set_error(error, n == 0 ? std::string("Device disconnected") : errno_text("read"));
        return -1;
    }
    if (pfd.revents & (POLLHUP | POLLERR)) {
        set_error(error, "Device disconnected");
        return -1;
    }
    return 0;
}

bool SerialPort::write_all(const uint8_t* data, size_t len, int timeout_ms, std::string* error) {
    if (fd_ < 0) {
        set_error(error, "Port is not open");
        return false;
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    size_t done = 0;
    while (done < len) {
        const ssize_t n = ::write(fd_, data + done, len - done);
        if (n > 0) {
            done += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            set_error(error, errno_text("write"));
            return false;
        }
        // Output buffer full: wait for room.
        const int left = remaining_ms(deadline);
        if (left <= 0) {
            set_error(error, "Write timed out");
            return false;
        }
        pollfd pfd{fd_, POLLOUT, 0};
        const int rc = ::poll(&pfd, 1, left);
        if (rc < 0 && errno != EINTR) {
            set_error(error, errno_text("poll"));
            return false;
        }
        if (rc > 0 && (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            set_error(error, "Device disconnected");
            return false;
        }
    }
    return true;
}

}  // namespace basestation
