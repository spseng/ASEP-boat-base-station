#pragma once

// Minimal POSIX serial port (termios). macOS and Linux only.
// Raw 8N1, no flow control, non-blocking fd driven by poll().

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace basestation {

struct SerialPortInfo {
    std::string path;         // e.g. /dev/ttyUSB0, /dev/cu.usbserial-0001
    std::string description;  // best-effort, may be empty
};

// Candidate ports: /dev/ttyUSB*, /dev/ttyACM* and /dev/serial/by-id/* on
// Linux; /dev/cu.* on macOS (cu.*, not tty.*, so open() does not wait for
// carrier detect). Sorted, de-duplicated.
std::vector<SerialPortInfo> list_serial_ports();

// Baud rates offered in the UI.
const std::vector<uint32_t>& common_baud_rates();

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Opens and configures the port. On failure returns false and sets *error.
    // Any path is accepted (e.g. a pty from the simulator).
    bool open(const std::string& path, uint32_t baud, std::string* error);
    void close();
    bool is_open() const { return fd_ >= 0; }
    const std::string& path() const { return path_; }
    uint32_t baud() const { return baud_; }

    // Waits up to timeout_ms for input. Returns bytes read (0 on timeout),
    // or -1 on an error that means the port is gone (e.g. USB unplugged);
    // *error is set in that case.
    long read(uint8_t* buf, size_t cap, int timeout_ms, std::string* error);

    // Writes every byte, waiting up to timeout_ms in total. False on error or
    // timeout (*error set).
    bool write_all(const uint8_t* data, size_t len, int timeout_ms, std::string* error);

private:
    int fd_ = -1;
    std::string path_;
    uint32_t baud_ = 0;
};

}  // namespace basestation
