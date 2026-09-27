#include <basestation/core/frame_log.h>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <unistd.h>

using namespace basestation;
namespace fs = std::filesystem;

namespace {

// A scratch directory removed when the test ends.
struct TempDir {
    fs::path path;
    TempDir() {
        static int counter = 0;
        path = fs::temp_directory_path() /
               ("asep-frame-log-test-" + std::to_string(::getpid()) + "-" + std::to_string(counter++));
        fs::remove_all(path);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
};

LogRecord make_record(int i) {
    LogRecord r;
    r.dir = (i % 3 == 0) ? Direction::Tx : Direction::Rx;
    r.unix_us = 1700000000000000LL + i * 12345 - (i == 4 ? 3000000000000000LL : 0);  // one negative
    r.frame.type = static_cast<uint8_t>(i * 7);
    r.frame.seq = static_cast<uint8_t>(255 - i);
    // Lengths 0, 1, ..., plus one maximal payload.
    r.frame.len = static_cast<uint8_t>(i == 5 ? wirelink::MAX_PAYLOAD : i * 3);
    r.frame.payload.len = r.frame.len;
    for (size_t k = 0; k < r.frame.len; ++k) r.frame.payload.data[k] = static_cast<uint8_t>(k * 31 + i);
    return r;
}

void check_same(const LogRecord& a, const LogRecord& b) {
    CHECK(a.dir == b.dir);
    CHECK(a.unix_us == b.unix_us);
    CHECK(a.frame.type == b.frame.type);
    CHECK(a.frame.seq == b.frame.seq);
    REQUIRE(a.frame.len == b.frame.len);
    CHECK(b.frame.payload.len == b.frame.len);
    CHECK(std::equal(a.frame.payload.data.begin(), a.frame.payload.data.begin() + a.frame.len,
                     b.frame.payload.data.begin()));
}

constexpr int N = 12;

std::string write_log(const TempDir& dir) {
    // Nested path: open() must create the parent directories.
    const std::string path = (dir.path / "a" / "b" / "session.aseplog").string();
    FrameLogWriter w;
    std::string err;
    REQUIRE(w.open(path, &err));
    CHECK(err.empty());
    CHECK(w.is_open());
    CHECK(w.path() == path);
    for (int i = 0; i < N; ++i) w.write(make_record(i));
    CHECK(w.records() == static_cast<uint64_t>(N));
    w.close();
    CHECK_FALSE(w.is_open());
    w.write(make_record(0));  // no-op when closed
    CHECK(w.records() == static_cast<uint64_t>(N));
    return path;
}

std::vector<uint8_t> read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

}  // namespace

TEST_CASE("frame log round-trips records", "[io]") {
    TempDir dir;
    const std::string path = write_log(dir);

    FrameLogReader r;
    std::string err;
    REQUIRE(r.open(path, &err));
    LogRecord rec;
    for (int i = 0; i < N; ++i) {
        REQUIRE(r.next(rec));
        check_same(make_record(i), rec);
    }
    CHECK_FALSE(r.next(rec));
}

TEST_CASE("frame log file layout matches the documented format", "[io]") {
    TempDir dir;
    const std::string path = write_log(dir);
    const auto bytes = read_file(path);

    size_t expected = 8;
    for (int i = 0; i < N; ++i) expected += 12 + make_record(i).frame.len;
    REQUIRE(bytes.size() == expected);
    CHECK(std::string(bytes.begin(), bytes.begin() + 8) == "ASEPLOG1");

    // Record 0: tx, t = 1700000000000000 = 0x00060A24181E4000, type 0, seq 255, len 0.
    const std::vector<uint8_t> rec0 = {1, 0x00, 0x40, 0x1E, 0x18, 0x24, 0x0A, 0x06, 0x00, 0, 255, 0};
    CHECK(std::vector<uint8_t>(bytes.begin() + 8, bytes.begin() + 20) == rec0);
    // Record 1 follows immediately: rx, type 7, seq 254, len 3.
    CHECK(bytes[20] == 0);
    CHECK(bytes[29] == 7);
    CHECK(bytes[30] == 254);
    CHECK(bytes[31] == 3);
}

TEST_CASE("frame log reader stops at a truncated record", "[io]") {
    TempDir dir;
    const std::string path = write_log(dir);
    fs::resize_file(path, fs::file_size(path) - 3);

    FrameLogReader r;
    REQUIRE(r.open(path, nullptr));
    LogRecord rec;
    int count = 0;
    while (r.next(rec)) {
        check_same(make_record(count), rec);
        ++count;
    }
    CHECK(count == N - 1);
}

TEST_CASE("frame log reader rejects a bad header", "[io]") {
    TempDir dir;
    fs::create_directories(dir.path);
    const std::string bad = (dir.path / "bad.aseplog").string();
    const std::string shrt = (dir.path / "short.aseplog").string();
    std::ofstream(bad, std::ios::binary) << "NOTALOG!rest";
    std::ofstream(shrt, std::ios::binary) << "ASEP";

    FrameLogReader r;
    std::string err;
    CHECK_FALSE(r.open(bad, &err));
    CHECK_FALSE(err.empty());
    err.clear();
    CHECK_FALSE(r.open(shrt, &err));
    CHECK_FALSE(err.empty());
    err.clear();
    CHECK_FALSE(r.open((dir.path / "missing.aseplog").string(), &err));
    CHECK_FALSE(err.empty());
    LogRecord rec;
    CHECK_FALSE(r.next(rec));
}

TEST_CASE("default log filename", "[io]") {
    const std::string name = default_log_filename();
    // asep-YYYYmmdd-HHMMSS.aseplog
    REQUIRE(name.size() == 5 + 8 + 1 + 6 + 8);
    CHECK(name.rfind("asep-", 0) == 0);
    CHECK(name.substr(name.size() - 8) == ".aseplog");
    CHECK(name[13] == '-');
}
