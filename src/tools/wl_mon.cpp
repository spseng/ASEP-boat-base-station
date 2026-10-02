// wl_mon: read-only monitor for any wirelink serial link, in the spirit of
// `ros2 topic echo` and `ros2 topic hz`. Decodes every frame with
// basestation::catalog and flags values that cannot be right.
//
// It never writes to the port, so it is safe to leave running next to
// firmware under test; if the port vanishes (ESP32 reset, unplug, flashing)
// it waits and reopens it.

#include <basestation/core/common.h>
#include <basestation/core/fleet_model.h>
#include <basestation/core/frame_splitter.h>
#include <basestation/core/msg_catalog.h>
#include <basestation/core/serial_port.h>

#include <boat_defs/serial.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <sys/ioctl.h>
#include <unistd.h>

namespace {

using namespace basestation;
using catalog::Decoded;
using catalog::Proto;
using catalog::Severity;

volatile std::sig_atomic_t g_stop = 0;

void on_signal(int) { g_stop = 1; }

struct Options {
    bool top = false;
    std::string port;
    Proto proto = Proto::Lora;
    uint32_t baud = boat::serial::BAUD_RATE;
    std::set<uint8_t> types;  // echo filter; empty = all
    bool color = true;
};

void usage(std::FILE* out) {
    std::fprintf(out,
        "usage: wl_mon echo PORT [options]   print every frame, one line each\n"
        "       wl_mon top  PORT [options]   live table per message type (~4 Hz)\n"
        "\n"
        "Read-only monitor for a wirelink serial link. Decodes every frame and flags\n"
        "values that cannot be right (BOGUS, red) or look suspicious (WARN, yellow).\n"
        "It never writes to the port. If the port disappears (reset, unplug,\n"
        "flashing) it waits and reopens it.\n"
        "\n"
        "  --proto lora|serial  message set (default lora):\n"
        "                         lora    the base-station ESP32's USB link: LoRa\n"
        "                                 messages (types 0-5) + base-local (0x80+)\n"
        "                         serial  a boat's Pi <-> ESP32 link (types 1-10)\n"
        "                       The two sets reuse type numbers (3 is Status on lora,\n"
        "                       PeerTable on serial), so pick the one for the link.\n"
        "  --baud N             baud rate (default %u)\n"
        "  --type NAME          echo: show only this message (repeatable)\n"
        "  --no-color           no colors (also off when stdout is not a terminal)\n"
        "  --help               show this help\n",
        unsigned(boat::serial::BAUD_RATE));
}

#if defined(__GNUC__)
__attribute__((format(printf, 1, 2)))
#endif
std::string fmt(const char* f, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    return buf;
}

// ---------------------------------------------------------------------------
// Terminal helpers (plain ANSI escapes)
// ---------------------------------------------------------------------------

const char* const RED = "31";
const char* const YELLOW = "33";
const char* const GREEN = "32";
const char* const DIM = "2";

bool g_color = false;

std::string paint(const std::string& s, const char* code) {
    return g_color ? "\x1b[" + std::string(code) + "m" + s + "\x1b[0m" : s;
}

struct TermSize {
    int cols = 120;
    int rows = 0;  // 0 = unknown
};

TermSize term_size() {
    TermSize t;
    winsize ws{};
    if (::ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_col > 0) {
        t.cols = ws.ws_col;
        t.rows = ws.ws_row;
    }
    return t;
}

// Cuts `s` to `cols` terminal columns, counting UTF-8 code points ("°C")
// and skipping colour escapes.
std::string truncate(const std::string& s, int cols) {
    int n = 0;
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '\x1b') {
            while (i < s.size() && !std::isalpha(static_cast<unsigned char>(s[i]))) ++i;
            continue;
        }
        if ((static_cast<unsigned char>(s[i]) & 0xC0) == 0x80) continue;
        if (++n > cols) return s.substr(0, i) + (g_color ? "\x1b[0m" : "");
    }
    return s;
}

// Local wall-clock time, HH:MM:SS.mmm.
std::string clock_text() {
    const int64_t us = unix_time_us();
    const std::time_t secs = static_cast<std::time_t>(us / 1000000);
    std::tm tm{};
    ::localtime_r(&secs, &tm);
    return fmt("%02d:%02d:%02d.%03d", tm.tm_hour, tm.tm_min, tm.tm_sec, int(us / 1000 % 1000));
}

std::string join_problems(const Decoded& d, Severity sev) {
    std::string s;
    for (const auto& p : d.problems)
        if (p.severity == sev) s += (s.empty() ? "" : "; ") + p.text;
    return s;
}

std::string fields_text(const Decoded& d) {
    std::string s;
    for (const auto& f : d.fields) s += (s.empty() ? "" : " ") + (f.name.empty() ? f.value : f.name + "=" + f.value);
    return s;
}

// " BOGUS: ... WARN: ..." in colour, or "".
std::string problems_text(const Decoded& d) {
    std::string s;
    const std::string err = join_problems(d, Severity::Error);
    const std::string warn = join_problems(d, Severity::Warning);
    if (!err.empty()) s += " " + paint("BOGUS: " + err, RED);
    if (!warn.empty()) s += " " + paint("WARN: " + warn, YELLOW);
    return s;
}

// ---------------------------------------------------------------------------
// Port: read-only, reopened whenever it disappears
// ---------------------------------------------------------------------------

class Port {
public:
    explicit Port(const Options& o) : opt_(o) {}

    // Waits up to timeout_ms for bytes and appends any complete frames.
    // Returns a status line when the link state changes, else "".
    std::string step(std::vector<wirelink::Frame>& frames, int timeout_ms) {
        if (!port_.is_open()) {
            const auto now = Clock::now();
            std::string err;
            if (now - last_try_ >= std::chrono::milliseconds(500)) {
                last_try_ = now;
                if (port_.open(opt_.port, opt_.baud, &err)) {
                    // A fresh splitter drops the old port's partial frame (no
                    // spurious CRC error on reconnect); its totals are kept.
                    add(prev_, splitter_.stats());
                    splitter_ = FrameSplitter{};
                    waiting_shown_.clear();
                    return "connected to " + opt_.port;
                }
            }
            const timespec ts{timeout_ms / 1000, (timeout_ms % 1000) * 1000000L};
            ::nanosleep(&ts, nullptr);  // returns early on Ctrl-C
            if (!err.empty() && err != waiting_shown_) {
                waiting_shown_ = err;
                return "waiting for " + opt_.port + "… (" + err + ")";
            }
            return {};
        }

        uint8_t buf[4096];
        std::string err;
        const long n = port_.read(buf, sizeof buf, timeout_ms, &err);
        if (n < 0) {
            port_.close();
            last_try_ = Clock::now();
            waiting_shown_.clear();
            return opt_.port + ": " + err;
        }
        if (n > 0) splitter_.feed(buf, static_cast<size_t>(n), frames);
        return {};
    }

    bool connected() const { return port_.is_open(); }

    FrameSplitter::Stats stats() const {
        FrameSplitter::Stats s = prev_;
        add(s, splitter_.stats());
        return s;
    }

private:
    static void add(FrameSplitter::Stats& a, const FrameSplitter::Stats& b) {
        a.bytes += b.bytes;
        a.frames_ok += b.frames_ok;
        a.frames_bad += b.frames_bad;
        a.overflows += b.overflows;
    }

    const Options& opt_;
    SerialPort port_;  // only ever read: wl_mon must not disturb the link
    FrameSplitter splitter_;
    FrameSplitter::Stats prev_{};
    Clock::time_point last_try_{};
    std::string waiting_shown_;
};

// Corrupt frames never reach the catalog, so report the splitter's counters.
struct BadCounter {
    uint64_t bad = 0;
    uint64_t overflows = 0;

    // Lines describing what went wrong since the last call.
    std::vector<std::string> update(const FrameSplitter::Stats& s) {
        std::vector<std::string> out;
        if (s.frames_bad > bad) {
            out.push_back(fmt("CRC/COBS failure: %llu frame(s) dropped (%llu total)",
                              static_cast<unsigned long long>(s.frames_bad - bad),
                              static_cast<unsigned long long>(s.frames_bad)));
        }
        if (s.overflows > overflows) {
            out.push_back(fmt("overflow: run longer than any frame, %llu total",
                              static_cast<unsigned long long>(s.overflows)));
        }
        bad = s.frames_bad;
        overflows = s.overflows;
        return out;
    }
};

void status_line(const std::string& s) {
    if (!s.empty()) std::fprintf(stderr, "%s  %s\n", clock_text().c_str(), s.c_str());
}

// ---------------------------------------------------------------------------
// echo
// ---------------------------------------------------------------------------

int run_echo(const Options& opt) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);  // one line per frame, promptly, even into a pipe
    Port port(opt);
    BadCounter bad;
    std::vector<wirelink::Frame> frames;
    while (!g_stop) {
        frames.clear();
        status_line(port.step(frames, 100));
        for (const auto& line : bad.update(port.stats()))
            std::printf("%s  %s\n", clock_text().c_str(), paint("BOGUS: " + line, RED).c_str());
        for (const auto& f : frames) {
            if (!opt.types.empty() && !opt.types.count(f.type)) continue;
            const Decoded d = catalog::decode(opt.proto, f);
            std::printf("%s  %-16s seq=%-3u  %s%s\n", clock_text().c_str(), d.name.c_str(), unsigned(f.seq),
                        fields_text(d).c_str(), problems_text(d).c_str());
        }
    }
    return 0;
}

// ---------------------------------------------------------------------------
// top
// ---------------------------------------------------------------------------

struct Row {
    std::string name;
    uint64_t count = 0;
    uint64_t bogus = 0;
    uint64_t warn = 0;
    std::map<int, SeqStats> seq;          // per sender (-1: message has none)
    std::deque<Clock::time_point> times;  // recent arrivals, for Hz
    Clock::time_point last{};
    Decoded latest;

    void add(const wirelink::Frame& f, Decoded d, Clock::time_point t) {
        ++count;
        if (d.has(Severity::Error)) ++bogus;
        if (d.has(Severity::Warning)) ++warn;
        seq[d.sender].update(f.seq);
        times.push_back(t);
        last = t;
        name = d.name;
        latest = std::move(d);
    }

    // Mean rate over the last 10 s, like `ros2 topic hz`.
    double hz(Clock::time_point now) {
        while (!times.empty() && now - times.front() > std::chrono::seconds(10)) times.pop_front();
        if (times.size() < 2) return 0;
        const double span = std::chrono::duration<double>(times.back() - times.front()).count();
        return span > 0 ? double(times.size() - 1) / span : 0;
    }

    uint64_t lost() const {
        uint64_t n = 0;
        for (const auto& [sender, s] : seq) n += s.lost;
        return n;
    }
};

class Top {
public:
    explicit Top(const Options& o) : opt_(o), port_(o) {}

    int run() {
        const bool tty = ::isatty(STDOUT_FILENO);
        // Alternate screen, cursor hidden; both undone on exit.
        if (tty) std::fputs("\x1b[?1049h\x1b[?25l", stdout);
        const auto period = tty ? std::chrono::milliseconds(250) : std::chrono::milliseconds(1000);
        auto next_draw = Clock::now();
        std::vector<wirelink::Frame> frames;
        while (!g_stop) {
            frames.clear();
            const std::string status = port_.step(frames, 50);
            const auto now = Clock::now();
            if (!status.empty()) {
                link_text_ = status;
                if (!tty) status_line(status);
            }
            for (const auto& line : bad_.update(port_.stats())) remember(paint("BOGUS: " + line, RED));
            for (const auto& f : frames) {
                Decoded d = catalog::decode(opt_.proto, f);
                if (!d.problems.empty())
                    remember(d.name + fmt(" seq=%u", unsigned(f.seq)) + problems_text(d));
                rows_[f.type].add(f, std::move(d), now);
            }
            if (now >= next_draw) {
                next_draw = now + period;
                draw(tty, now);
            }
        }
        if (tty) std::fputs("\x1b[?25h\x1b[?1049l", stdout);
        std::fflush(stdout);
        return 0;
    }

private:
    void remember(const std::string& s) {
        recent_.push_back(clock_text() + "  " + s);
        if (recent_.size() > 5) recent_.pop_front();
    }

    void draw(bool tty, Clock::time_point now) {
        const TermSize ts = tty ? term_size() : TermSize{};
        const int width = ts.cols;
        const FrameSplitter::Stats st = port_.stats();

        // Bytes per second over the last redraw interval(s), at most every second.
        if (now - rate_t_ >= std::chrono::seconds(1)) {
            const double dt = std::chrono::duration<double>(now - rate_t_).count();
            if (rate_t_ != Clock::time_point{}) bps_ = double(st.bytes - rate_bytes_) / dt;
            rate_t_ = now;
            rate_bytes_ = st.bytes;
        }

        std::vector<std::string> lines;
        const std::string link =
            port_.connected() ? paint("connected", GREEN)
                              : paint(link_text_.empty() ? "waiting for " + opt_.port + "…" : link_text_, YELLOW);
        lines.push_back(fmt("wl_mon  %s  proto=%s  baud=%u  ", opt_.port.c_str(), catalog::proto_name(opt_.proto),
                            unsigned(opt_.baud)) + link +
                        fmt("  %.0f B/s  frames=%llu  CRC-bad=", bps_, static_cast<unsigned long long>(st.frames_ok)) +
                        count_cell(st.frames_bad, RED, 0) + "  overflow=" + count_cell(st.overflows, RED, 0));
        lines.push_back("");
        const std::string head = fmt("%-21s %7s %6s %5s %5s %5s %6s  %s", "Type", "Count", "Hz", "Lost", "Bogus",
                                     "Warn", "Age", "Latest");
        lines.push_back(paint(truncate(head, width), DIM));
        const int fixed = 21 + 1 + 7 + 1 + 6 + 1 + 5 + 1 + 5 + 1 + 5 + 1 + 6 + 2;  // visible width before Latest
        for (auto& [type, r] : rows_) {
            const double age = std::chrono::duration<double>(now - r.last).count();
            std::string latest = fields_text(r.latest);
            const char* latest_color = nullptr;
            if (r.latest.has(Severity::Warning)) latest_color = YELLOW;
            if (r.latest.has(Severity::Error)) latest_color = RED;
            if (!r.latest.problems.empty()) latest += " !! " + problems_plain(r.latest);
            latest = truncate(latest, width - fixed);
            std::string line = fmt("0x%02X %-16s %7llu %6.1f %5llu ", unsigned(type), truncate(r.name, 16).c_str(),
                                   static_cast<unsigned long long>(r.count), r.hz(now),
                                   static_cast<unsigned long long>(r.lost())) +
                               count_cell(r.bogus, RED, 5) + " " + count_cell(r.warn, YELLOW, 5) +
                               fmt(" %6s  ", age_text(age).c_str()) +
                               (latest_color ? paint(latest, latest_color) : latest);
            lines.push_back(line);
        }
        if (rows_.empty()) lines.push_back("(no frames yet)");
        if (!recent_.empty()) {
            lines.push_back("");
            lines.push_back(paint("Recent problems:", DIM));
            for (const auto& s : recent_) lines.push_back(s);
        }

        if (tty) {
            // Redraw in place: home, overwrite each line, clear the rest.
            std::string out = "\x1b[H";
            const size_t max_lines = ts.rows > 0 ? static_cast<size_t>(ts.rows) : lines.size();
            for (size_t i = 0; i < lines.size() && i < max_lines; ++i) {
                // Never wrap: a wrapped line would scroll the screen.
                out += truncate(lines[i], width) + "\x1b[K" + (i + 1 < std::min(lines.size(), max_lines) ? "\n" : "");
            }
            out += "\x1b[J";
            std::fputs(out.c_str(), stdout);
        } else {
            std::printf("--- %s\n", clock_text().c_str());
            for (const auto& l : lines) std::printf("%s\n", l.c_str());
            std::printf("\n");
        }
        std::fflush(stdout);
    }

    static std::string problems_plain(const Decoded& d) {
        std::string s;
        for (const auto& p : d.problems) s += (s.empty() ? "" : "; ") + p.text;
        return s;
    }

    static std::string count_cell(uint64_t n, const char* color, int w) {
        const std::string s = fmt("%*llu", w, static_cast<unsigned long long>(n));
        return n ? paint(s, color) : s;
    }

    static std::string age_text(double s) {
        if (s < 10) return fmt("%.1fs", s);
        if (s < 600) return fmt("%.0fs", s);
        return fmt("%.0fm", s / 60);
    }

    const Options& opt_;
    Port port_;
    BadCounter bad_;
    std::map<uint8_t, Row> rows_;  // sorted by type number
    std::deque<std::string> recent_;
    std::string link_text_;
    Clock::time_point rate_t_{};
    uint64_t rate_bytes_ = 0;
    double bps_ = 0;
};

// ---------------------------------------------------------------------------

bool parse_args(int argc, char** argv, Options& o, std::vector<std::string>& type_names) {
    std::vector<std::string> pos;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto value = [&](std::string& out) {
            if (i + 1 >= argc) return false;
            out = argv[++i];
            return true;
        };
        std::string v;
        if (a == "--proto") {
            if (!value(v) || !catalog::parse_proto(v, o.proto)) return false;
        } else if (a == "--baud") {
            if (!value(v)) return false;
            char* end = nullptr;
            const unsigned long b = std::strtoul(v.c_str(), &end, 10);
            if (*end || b == 0 || b > 10000000) return false;
            o.baud = static_cast<uint32_t>(b);
        } else if (a == "--type") {
            if (!value(v)) return false;
            type_names.push_back(v);
        } else if (a == "--no-color") {
            o.color = false;
        } else if (!a.empty() && a[0] == '-') {
            return false;
        } else {
            pos.push_back(a);
        }
    }
    if (pos.size() != 2 || (pos[0] != "echo" && pos[0] != "top")) return false;
    o.top = pos[0] == "top";
    o.port = pos[1];
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--help") || !std::strcmp(argv[i], "-h")) {
            usage(stdout);
            return 0;
        }
    }
    Options opt;
    std::vector<std::string> type_names;
    if (!parse_args(argc, argv, opt, type_names)) {
        usage(stderr);
        return 2;
    }
    for (const auto& n : type_names) {
        const catalog::MsgSpec* m = catalog::find(opt.proto, n);
        if (!m) {
            std::fprintf(stderr, "wl_mon: no message '%s' in --proto %s; known:", n.c_str(),
                         catalog::proto_name(opt.proto));
            for (const auto& k : catalog::messages(opt.proto)) std::fprintf(stderr, " %s", k.name);
            std::fprintf(stderr, "\n");
            return 2;
        }
        opt.types.insert(m->type);
    }
    g_color = opt.color && ::isatty(STDOUT_FILENO);

    // No SA_RESTART: poll()/nanosleep() must return so the loop sees g_stop.
    struct sigaction sa {};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    ::sigaction(SIGINT, &sa, nullptr);
    ::sigaction(SIGTERM, &sa, nullptr);
    ::sigaction(SIGHUP, &sa, nullptr);
    std::signal(SIGPIPE, SIG_IGN);

    return opt.top ? Top(opt).run() : run_echo(opt);
}
