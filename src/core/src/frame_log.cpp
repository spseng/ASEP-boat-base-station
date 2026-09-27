#include <basestation/core/frame_log.h>

#include <cerrno>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <system_error>

namespace basestation {

namespace {

constexpr char MAGIC[8] = {'A', 'S', 'E', 'P', 'L', 'O', 'G', '1'};
// direction + unix_us + type + seq + len
constexpr size_t RECORD_HEADER = 1 + 8 + 1 + 1 + 1;

void set_error(std::string* error, std::string text) {
    if (error) *error = std::move(text);
}

}  // namespace

std::string default_log_filename() {
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    char buf[64];
    if (std::strftime(buf, sizeof buf, "asep-%Y%m%d-%H%M%S.aseplog", &local) == 0) return "asep.aseplog";
    return buf;
}

// --- Writer -----------------------------------------------------------------

FrameLogWriter::~FrameLogWriter() { close(); }

bool FrameLogWriter::open(const std::string& path, std::string* error) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_) {
        std::fclose(file_);
        file_ = nullptr;
    }
    records_ = 0;

    const std::filesystem::path parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(parent, ec);
        if (ec) {
            set_error(error, "Cannot create " + parent.string() + ": " + ec.message());
            return false;
        }
    }

    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
        set_error(error, "Cannot open " + path + ": " + std::strerror(errno));
        return false;
    }
    if (std::fwrite(MAGIC, 1, sizeof MAGIC, f) != sizeof MAGIC || std::fflush(f) != 0) {
        set_error(error, "Cannot write " + path + ": " + std::strerror(errno));
        std::fclose(f);
        return false;
    }
    file_ = f;
    path_ = path;
    return true;
}

void FrameLogWriter::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_) {
        std::fclose(file_);
        file_ = nullptr;
    }
}

bool FrameLogWriter::is_open() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return file_ != nullptr;
}

std::string FrameLogWriter::path() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return path_;
}

uint64_t FrameLogWriter::records() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return records_;
}

void FrameLogWriter::write(const LogRecord& rec) {
    // Serialised field by field so the file layout never depends on struct
    // padding or host byte order.
    uint8_t buf[RECORD_HEADER + wirelink::MAX_PAYLOAD];
    size_t n = 0;
    buf[n++] = static_cast<uint8_t>(rec.dir);
    const uint64_t t = static_cast<uint64_t>(rec.unix_us);
    for (int i = 0; i < 8; ++i) buf[n++] = static_cast<uint8_t>(t >> (8 * i));
    const uint8_t len = rec.frame.len > wirelink::MAX_PAYLOAD
        ? static_cast<uint8_t>(wirelink::MAX_PAYLOAD) : rec.frame.len;
    buf[n++] = rec.frame.type;
    buf[n++] = rec.frame.seq;
    buf[n++] = len;
    for (size_t i = 0; i < len; ++i) buf[n++] = rec.frame.payload.data[i];

    std::lock_guard<std::mutex> lock(mutex_);
    if (!file_) return;
    // Flushed per record so a crash loses at most the record in flight;
    // frame rates here are low enough that this costs nothing.
    std::fwrite(buf, 1, n, file_);
    std::fflush(file_);
    ++records_;
}

// --- Reader -----------------------------------------------------------------

FrameLogReader::~FrameLogReader() { close(); }

bool FrameLogReader::open(const std::string& path, std::string* error) {
    close();
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) {
        set_error(error, "Cannot open " + path + ": " + std::strerror(errno));
        return false;
    }
    char magic[sizeof MAGIC];
    if (std::fread(magic, 1, sizeof magic, f) != sizeof magic ||
        std::memcmp(magic, MAGIC, sizeof MAGIC) != 0) {
        set_error(error, path + " is not an ASEP session log");
        std::fclose(f);
        return false;
    }
    file_ = f;
    return true;
}

bool FrameLogReader::next(LogRecord& out) {
    if (!file_) return false;
    uint8_t head[RECORD_HEADER];
    if (std::fread(head, 1, sizeof head, file_) != sizeof head) return false;
    if (head[0] > static_cast<uint8_t>(Direction::Tx)) return false;  // corrupt

    LogRecord rec{};
    rec.dir = static_cast<Direction>(head[0]);
    uint64_t t = 0;
    for (int i = 0; i < 8; ++i) t |= static_cast<uint64_t>(head[1 + i]) << (8 * i);
    rec.unix_us = static_cast<int64_t>(t);
    rec.frame.type = head[9];
    rec.frame.seq = head[10];
    rec.frame.len = head[11];
    if (rec.frame.len > wirelink::MAX_PAYLOAD) return false;  // corrupt
    if (std::fread(rec.frame.payload.data.data(), 1, rec.frame.len, file_) != rec.frame.len) return false;
    rec.frame.payload.len = rec.frame.len;
    out = rec;
    return true;
}

void FrameLogReader::close() {
    if (file_) {
        std::fclose(file_);
        file_ = nullptr;
    }
}

}  // namespace basestation
