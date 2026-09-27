#pragma once

// Session recording: every frame received from and sent to the base
// station, with wall-clock timestamps, so a session can be replayed or
// analysed afterwards (tools/asep_log_dump converts to CSV).
//
// File format (little-endian):
//   header  : 8 bytes  "ASEPLOG1"
//   record  : u8  direction (0 = rx, 1 = tx)
//             i64 unix time, microseconds
//             u8  type
//             u8  seq
//             u8  len
//             len payload bytes
// A truncated final record (e.g. after a crash) is ignored by the reader.

#include <basestation/core/common.h>

#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>

namespace basestation {

enum class Direction : uint8_t { Rx = 0, Tx = 1 };

struct LogRecord {
    Direction dir = Direction::Rx;
    int64_t unix_us = 0;
    wirelink::Frame frame{};
};

// "asep-YYYYmmdd-HHMMSS.aseplog" in local time.
std::string default_log_filename();

class FrameLogWriter {
public:
    FrameLogWriter() = default;
    ~FrameLogWriter();
    FrameLogWriter(const FrameLogWriter&) = delete;
    FrameLogWriter& operator=(const FrameLogWriter&) = delete;

    // Creates/truncates `path` (parent directories are created).
    bool open(const std::string& path, std::string* error);
    void close();
    bool is_open() const;
    std::string path() const;
    uint64_t records() const;

    // Thread-safe. No-op when closed.
    void write(const LogRecord& rec);
    void write(Direction dir, const wirelink::Frame& frame) {
        write(LogRecord{dir, unix_time_us(), frame});
    }

private:
    mutable std::mutex mutex_;
    std::FILE* file_ = nullptr;
    std::string path_;
    uint64_t records_ = 0;
};

class FrameLogReader {
public:
    FrameLogReader() = default;
    ~FrameLogReader();
    FrameLogReader(const FrameLogReader&) = delete;
    FrameLogReader& operator=(const FrameLogReader&) = delete;

    bool open(const std::string& path, std::string* error);  // checks header
    bool next(LogRecord& out);  // false at end of file or truncated record
    void close();

private:
    std::FILE* file_ = nullptr;
};

}  // namespace basestation
