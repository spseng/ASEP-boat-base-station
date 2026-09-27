// asep_fake_base: pretends to be the base-station ESP32 plus a fleet of
// boats, on a pseudo-terminal, so the UI and tests run without hardware.
//
// Deliberately independent of basestation_core: incoming bytes are parsed
// with upstream wirelink::Link and messages are encoded with
// basestation::codec, so a bug in core cannot hide behind a matching bug here.

#include "sim_world.h"

#include <basestation/proto/base.h>
#include <basestation/proto/codec.h>
#include <basestation/proto/lora.h>
#include <basestation/proto/names.h>

#include <boat_defs/ids.h>
#include <boat_defs/mode.h>
#include <boat_defs/units.h>

#include <wirelink/framing.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdarg>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>

#include <fcntl.h>
#include <poll.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#if defined(__APPLE__)
#include <util.h>
#else
#include <pty.h>
#endif

namespace {

using boat::mode::ArmedState;
using boat::mode::GateState;
using boat::mode::Mode;
namespace lora = basestation::lora;
namespace base = basestation::base;
namespace codec = basestation::codec;
namespace names = basestation::names;

volatile std::sig_atomic_t g_stop = 0;

void on_signal(int) { g_stop = 1; }

struct Options {
    int boats = 3;
    uint32_t seed = 0;
    bool seed_set = false;
    double loss = 0.05;
    double corrupt = 0.002;
    double rate_hz = 3.0;
    sim::Origin origin;
    std::string link;
    bool quiet = false;
    bool rxinfo = true;
    bool basestatus = true;
    bool base_gps = false;     // --base-gps
    bool start_auto = false;   // --start-mode auto
    double time_scale = 1.0;   // --time-scale
};

void usage(std::FILE* out) {
    std::fprintf(out,
        "usage: asep_fake_base [options]\n"
        "\n"
        "Simulates the base-station ESP32 and a boat fleet on a pseudo-terminal.\n"
        "The first line on stdout is 'PORT <path>': open that path in the UI.\n"
        "\n"
        "  --boats N        number of boats, ids 1..N (default 3, max %d)\n"
        "  --seed S         RNG seed (default: random)\n"
        "  --loss P         probability a LoRa packet is lost, each way (default 0.05)\n"
        "  --corrupt P      probability a relayed frame gets a flipped byte (default 0.002)\n"
        "  --rate HZ        SelfStatus rate per boat (default 3)\n"
        "  --lat DEG        origin latitude  (default 42.3160, Jamaica Pond)\n"
        "  --lon DEG        origin longitude (default -71.1205)\n"
        "  --link PATH      also create a symlink PATH -> pty (removed on exit)\n"
        "  --quiet          do not print received commands\n"
        "  --no-rxinfo      do not send base::RxInfo after relayed frames\n"
        "  --no-basestatus  do not send base::BaseStatus\n"
        "  --base-gps       send base::BasePosition at 1 Hz, as if the base ESP32 had a\n"
        "                   GPS: no fix for the first second, then the origin +- ~1 m\n"
        "  --start-mode M   manual (default: MANUAL + DISARMED, boats drift) or\n"
        "                   auto (AUTONOMOUS + ARMED, boats wander; for demos/screenshots)\n"
        "  --time-scale F   run boat motion F times faster than real time (default 1;\n"
        "                   radio timing is unchanged; for demos/screenshots)\n"
        "  --help           show this help\n",
        static_cast<int>(boat::ids::BOAT_ID_MAX));
}

bool parse_double(const char* s, double lo, double hi, double& out) {
    char* end = nullptr;
    errno = 0;
    const double v = std::strtod(s, &end);
    if (errno || end == s || *end || !(v >= lo && v <= hi)) return false;
    out = v;
    return true;
}

// Returns 0 to run, otherwise an exit code + 1 (help -> 1, error -> 3).
int parse_args(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto value = [&](const char*& v) {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "asep_fake_base: %s needs a value\n", a.c_str());
                return false;
            }
            v = argv[++i];
            return true;
        };
        const char* v = nullptr;
        double d = 0;
        bool ok = true;
        if (a == "--help" || a == "-h") {
            usage(stdout);
            return 1;
        } else if (a == "--quiet") {
            o.quiet = true;
        } else if (a == "--no-rxinfo") {
            o.rxinfo = false;
        } else if (a == "--no-basestatus") {
            o.basestatus = false;
        } else if (a == "--base-gps") {
            o.base_gps = true;
        } else if (a == "--start-mode") {
            ok = value(v);
            if (ok && std::strcmp(v, "auto") == 0) o.start_auto = true;
            else if (ok && std::strcmp(v, "manual") == 0) o.start_auto = false;
            else ok = false;
        } else if (a == "--time-scale") {
            ok = value(v) && parse_double(v, 0.1, 100, o.time_scale);
        } else if (a == "--boats") {
            ok = value(v) && parse_double(v, 1, boat::ids::BOAT_ID_MAX, d) && d == static_cast<int>(d);
            o.boats = static_cast<int>(d);
        } else if (a == "--seed") {
            ok = value(v) && parse_double(v, 0, 4294967295.0, d);
            o.seed = static_cast<uint32_t>(d);
            o.seed_set = true;
        } else if (a == "--loss") {
            ok = value(v) && parse_double(v, 0, 1, o.loss);
        } else if (a == "--corrupt") {
            ok = value(v) && parse_double(v, 0, 1, o.corrupt);
        } else if (a == "--rate") {
            ok = value(v) && parse_double(v, 0.1, 50, o.rate_hz);
        } else if (a == "--lat") {
            ok = value(v) && parse_double(v, -89, 89, o.origin.lat_deg);
        } else if (a == "--lon") {
            ok = value(v) && parse_double(v, -180, 180, o.origin.lon_deg);
        } else if (a == "--link") {
            ok = value(v);
            if (ok) o.link = v;
        } else {
            std::fprintf(stderr, "asep_fake_base: unknown option %s\n", a.c_str());
            usage(stderr);
            return 3;
        }
        if (!ok) {
            if (v) std::fprintf(stderr, "asep_fake_base: bad value for %s: %s\n", a.c_str(), v);
            return 3;
        }
    }
    return 0;
}

std::string target(uint8_t rx_id) {
    return rx_id == boat::ids::BROADCAST_ID ? "ALL" : std::to_string(rx_id);
}

class FakeBase {
public:
    FakeBase(const Options& opt, int fd)
        : opt_(opt), fd_(fd), world_(opt.boats, opt.seed) {
        // TDMA-ish: each boat owns an equal slice of one SelfStatus period.
        const double period = 1.0 / opt_.rate_hz;
        const double slot = period / opt_.boats;
        for (size_t i = 0; i < world_.boats().size(); ++i) {
            sim::SimBoat& b = world_.boats()[i];
            b.next_slot_s = 0.05 + slot * static_cast<double>(i);
            b.next_status_s = b.next_slot_s;
            if (opt_.start_auto) {
                b.mode = Mode::AUTONOMOUS;
                b.armed = ArmedState::ARMED;
            }
        }
    }

    void run() {
        const auto t0 = std::chrono::steady_clock::now();
        uint8_t buf[512];
        while (!g_stop) {
            pollfd p{fd_, POLLIN, 0};
            const int r = ::poll(&p, 1, 5);
            if (r < 0 && errno != EINTR) {
                std::perror("asep_fake_base: poll");
                return;
            }
            if (r > 0 && (p.revents & POLLIN)) {
                const ssize_t n = ::read(fd_, buf, sizeof buf);
                for (ssize_t i = 0; i < n; ++i) {
                    if (auto f = rx_link_.feed(buf[i])) handle_land_frame(*f);
                }
            } else if (r > 0 && (p.revents & (POLLHUP | POLLERR))) {
                // No slave side (should not happen: we hold it open). Avoid spinning.
                timespec ts{0, 5'000'000};
                ::nanosleep(&ts, nullptr);
            }
            const double now = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            tick(now);
        }
    }

private:
    void tick(double now) {
        now_ = now;
        // Boat motion (and the fault schedule) runs on scaled "world" time;
        // radio slots and BaseStatus stay on real time.
        world_now_ = now * opt_.time_scale;
        world_.step(world_now_);
        world_.maybe_toggle_fault(world_now_);

        const double period = 1.0 / opt_.rate_hz;
        for (sim::SimBoat& b : world_.boats()) {
            if (now < b.next_slot_s) continue;
            send_self_status(b);
            if (now >= b.next_status_s || b.status_now) {
                send_status(b);
                b.status_now = false;
                b.next_status_s = std::max(b.next_status_s + 1.0, now + 0.5);
            }
            b.next_slot_s += period;
            if (b.next_slot_s < now) b.next_slot_s = now + period;  // fell behind
        }

        if (opt_.basestatus && now >= next_base_s_) {
            base::BaseStatus s{};
            s.uptime_ms = static_cast<uint32_t>(now * 1000.0);
            s.rx_ok = rx_ok_;
            s.rx_bad = rx_bad_;
            s.tx_count = tx_count_;
            send_local(s);
            next_base_s_ += 1.0;
        }

        if (opt_.base_gps && now >= next_gps_s_) {
            // The radio model puts the base station at the origin; report
            // that with a little GPS noise once the receiver has a fix.
            base::BasePosition p{};
            if (now >= 1.0) {
                std::normal_distribution<double> noise(0.0, 0.7);
                const auto g = to_geo(noise(world_.rng()), noise(world_.rng()));
                p.lat = g.lat;
                p.lon = g.lon;
                p.fix_quality = 1;
                p.satellites = static_cast<uint8_t>(std::uniform_int_distribution<int>(8, 12)(world_.rng()));
            } else {
                p.satellites = 2;
            }
            send_local(p);
            next_gps_s_ += 1.0;
        }
    }

    struct GeoE7 { int32_t lat, lon; };

    // Equirectangular local -> geodetic; plenty for a few hundred metres.
    GeoE7 to_geo(double north_m, double east_m) const {
        constexpr double R = 6371008.8;
        constexpr double DEG = 180.0 / 3.14159265358979323846;
        const double lat0 = opt_.origin.lat_deg;
        const double lat = lat0 + north_m / R * DEG;
        const double lon = opt_.origin.lon_deg + east_m / (R * std::cos(lat0 / DEG)) * DEG;
        return {boat::units::deg_to_e7(lat), boat::units::deg_to_e7(lon)};
    }

    void send_self_status(sim::SimBoat& b) {
        lora::SelfStatus m{};
        m.self.id = b.id;
        const auto g = to_geo(b.fix_north_m, b.fix_east_m);
        m.self.lat = g.lat;
        m.self.lon = g.lon;
        m.self.scalar = world_.scalar_at(b.north_m, b.east_m);
        m.self.age_ms = static_cast<uint32_t>((world_now_ - b.fix_time_s) / opt_.time_scale * 1000.0);
        send_from_boat(b, m);
    }

    void send_status(sim::SimBoat& b) {
        lora::Status m{};
        m.tx_id = b.id;
        m.mode = static_cast<uint8_t>(b.mode);
        m.armed = static_cast<uint8_t>(b.armed);
        m.gate_state = static_cast<uint8_t>(b.gate);
        m.fault_flags = b.fault_flags;
        m.heading_deg = static_cast<float>(b.heading_deg);
        m.gs_rssi = b.gs_rssi;
        m.gs_snr = b.gs_snr;
        send_from_boat(b, m);
    }

    // A boat transmits; the base station may or may not receive it.
    template <class T>
    void send_from_boat(sim::SimBoat& b, const T& msg) {
        wirelink::Frame f{};
        if (!codec::pack(msg, f)) return;
        // The seq is consumed even when the packet is lost, so the UI sees gaps.
        const uint8_t seq = b.seq[f.type]++;
        if (world_.chance(opt_.loss)) {
            // Some losses are CRC failures the base station notices.
            if (world_.chance(0.3)) ++rx_bad_;
            return;
        }
        ++rx_ok_;
        write_frame(f, seq, true);
        if (opt_.rxinfo) {
            base::RxInfo info{};
            world_.radio_at(b.north_m, b.east_m, info.rssi, info.snr);
            send_local(info);
        }
    }

    template <class T>
    void send_local(const T& msg) {
        wirelink::Frame f{};
        if (!codec::pack(msg, f)) return;
        write_frame(f, base_seq_[f.type]++, false);
    }

    void write_frame(const wirelink::Frame& f, uint8_t seq, bool corruptible) {
        auto pkt = wirelink::framing::wrap(f, seq);
        if (!pkt) return;
        if (corruptible && pkt->len > 2 && world_.chance(opt_.corrupt)) {
            // Flip bits in one encoded byte. Encoded bytes are never 0x00 and
            // must stay non-zero, or the frame boundaries would move.
            std::uniform_int_distribution<size_t> pos(1, pkt->len - 2);
            std::uniform_int_distribution<int> mask(1, 255);
            const size_t i = pos(world_.rng());
            uint8_t m;
            do { m = static_cast<uint8_t>(mask(world_.rng())); } while (m == pkt->data[i]);
            pkt->data[i] ^= m;
        }
        write_all(pkt->data.data(), pkt->len);
    }

    void write_all(const uint8_t* p, size_t n) {
        while (n > 0) {
            const ssize_t w = ::write(fd_, p, n);
            if (w > 0) {
                p += w;
                n -= static_cast<size_t>(w);
            } else if (w < 0 && errno == EINTR) {
                continue;
            } else {
                // Nobody draining the pty (buffer full): drop, like a UART
                // with no host attached. COBS resynchronises the reader.
                return;
            }
        }
    }

    // Land -> boat: which boats hear it is decided independently per boat.
    void handle_land_frame(const wirelink::Frame& f) {
        if (base::is_local_type(f.type)) {
            log_rx("RX local type=0x%02x len=%u (ignored)\n", unsigned(f.type), unsigned(f.len));
            return;
        }
        ++tx_count_;  // the base station transmits it over LoRa

        switch (static_cast<lora::MsgType>(f.type)) {
            case lora::MsgType::Disable: {
                lora::Disable m{};
                if (!codec::unpack(f, m)) break;
                const std::string heard = deliver(m.rx_id, [](sim::SimBoat& b) {
                    if (b.gate != GateState::TRIPPED) b.status_now = true;
                    b.gate = GateState::TRIPPED;
                });
                log_rx("RX Disable rx=%s seq=%u heard=%s\n", target(m.rx_id).c_str(), unsigned(f.seq), heard.c_str());
                return;
            }
            case lora::MsgType::Reenable: {
                lora::Reenable m{};
                if (!codec::unpack(f, m)) break;
                const std::string heard = deliver(m.rx_id, [](sim::SimBoat& b) {
                    if (b.gate != GateState::ENABLED) b.status_now = true;
                    b.gate = GateState::ENABLED;
                });
                log_rx("RX Reenable rx=%s seq=%u heard=%s\n", target(m.rx_id).c_str(), unsigned(f.seq), heard.c_str());
                return;
            }
            case lora::MsgType::SetMode: {
                lora::SetMode m{};
                if (!codec::unpack(f, m)) break;
                // Any mode value is accepted (newer firmware may know modes
                // this build does not); unknown modes hold still, like
                // EMERGENCY_STOP (see sim_world.cpp). Armed is binary.
                const bool valid = m.armed <= static_cast<uint8_t>(ArmedState::ARMED);
                const std::string heard = !valid ? "-" : deliver(m.rx_id, [&](sim::SimBoat& b) {
                    b.mode = static_cast<Mode>(m.mode);
                    b.armed = static_cast<ArmedState>(m.armed);
                    b.status_now = true;
                });
                log_rx("RX SetMode rx=%s mode=%s armed=%s seq=%u heard=%s%s\n",
                       target(m.rx_id).c_str(), names::mode_name(m.mode).c_str(),
                       names::armed_name(m.armed).c_str(), unsigned(f.seq), heard.c_str(),
                       valid ? "" : " (invalid, ignored)");
                return;
            }
            case lora::MsgType::Command: {
                lora::Command m{};
                if (!codec::unpack(f, m)) break;
                const std::string heard = deliver(m.rx_id, [&](sim::SimBoat& b) {
                    b.cmd_lin_mm = m.lin_vel;
                    b.cmd_ang_mrad = m.ang_vel;
                    b.cmd_time_s = world_now_;
                });
                log_rx("RX Command rx=%s lin=%dmm/s ang=%dmrad/s seq=%u heard=%s\n",
                       target(m.rx_id).c_str(), int(m.lin_vel), int(m.ang_vel), unsigned(f.seq), heard.c_str());
                return;
            }
            default:
                log_rx("RX type=%u len=%u seq=%u (not a land->boat message, ignored)\n",
                       unsigned(f.type), unsigned(f.len), unsigned(f.seq));
                return;
        }
        log_rx("RX type=%u len=%u seq=%u (payload did not decode)\n", unsigned(f.type), unsigned(f.len), unsigned(f.seq));
    }

    // Applies `fn` to every addressed boat that hears the packet; returns
    // their ids ("1,3", or "-" if none).
    template <class Fn>
    std::string deliver(uint8_t rx_id, Fn fn) {
        std::string heard;
        for (sim::SimBoat& b : world_.boats()) {
            if (rx_id != boat::ids::BROADCAST_ID && rx_id != b.id) continue;
            if (world_.chance(opt_.loss)) continue;
            world_.radio_at(b.north_m, b.east_m, b.gs_rssi, b.gs_snr);
            fn(b);
            if (!heard.empty()) heard += ',';
            heard += std::to_string(b.id);
        }
        return heard.empty() ? "-" : heard;
    }

#if defined(__GNUC__)
    __attribute__((format(printf, 2, 3)))
#endif
    void log_rx(const char* fmt, ...) {
        if (opt_.quiet) return;
        va_list ap;
        va_start(ap, fmt);
        std::vprintf(fmt, ap);
        va_end(ap);
    }

    const Options& opt_;
    int fd_;
    sim::World world_;
    wirelink::Link rx_link_;
    uint8_t base_seq_[256] = {};
    uint32_t rx_ok_ = 0;
    uint32_t rx_bad_ = 0;
    uint32_t tx_count_ = 0;
    double now_ = 0;
    double world_now_ = 0;
    double next_base_s_ = 0.5;
    double next_gps_s_ = 0.3;
};

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (const int rc = parse_args(argc, argv, opt)) return rc - 1;
    if (!opt.seed_set) opt.seed = std::random_device{}();

    int master = -1, slave = -1;
    if (::openpty(&master, &slave, nullptr, nullptr, nullptr) != 0) {
        std::perror("asep_fake_base: openpty");
        return 1;
    }
    // Raw on the slave so the client sees bytes unchanged even before it
    // configures the port. We keep the slave open for our whole life: the
    // master would otherwise read EIO whenever the client disconnects.
    termios tio{};
    if (::tcgetattr(slave, &tio) == 0) {
        ::cfmakeraw(&tio);
        ::tcsetattr(slave, TCSANOW, &tio);
    }
    ::fcntl(master, F_SETFL, ::fcntl(master, F_GETFL) | O_NONBLOCK);
    ::fcntl(master, F_SETFD, FD_CLOEXEC);
    ::fcntl(slave, F_SETFD, FD_CLOEXEC);
    const char* name = ::ttyname(slave);
    if (!name) {
        std::perror("asep_fake_base: ttyname");
        return 1;
    }
    const std::string slave_path = name;

    // No SA_RESTART: poll() must return so the loop sees g_stop.
    struct sigaction sa {};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    ::sigaction(SIGINT, &sa, nullptr);
    ::sigaction(SIGTERM, &sa, nullptr);
    ::sigaction(SIGHUP, &sa, nullptr);
    std::signal(SIGPIPE, SIG_IGN);  // a closed stdout pipe must not kill us

    bool linked = false;
    if (!opt.link.empty()) {
        struct stat st {};
        // Replace a stale symlink from an earlier run, never a real file.
        if (::lstat(opt.link.c_str(), &st) == 0 && S_ISLNK(st.st_mode)) ::unlink(opt.link.c_str());
        if (::symlink(slave_path.c_str(), opt.link.c_str()) == 0) {
            linked = true;
        } else {
            std::fprintf(stderr, "asep_fake_base: symlink %s: %s\n", opt.link.c_str(), std::strerror(errno));
        }
    }

    // Line-buffered so a parent reading our pipe sees each line promptly.
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    std::printf("PORT %s\n", slave_path.c_str());
    std::fflush(stdout);
    if (!opt.quiet) {
        std::fprintf(stderr,
            "asep_fake_base: %d boats, seed %u, loss %.3f, corrupt %.4f, rate %.1f Hz, origin %.6f,%.6f%s%s\n",
            opt.boats, opt.seed, opt.loss, opt.corrupt, opt.rate_hz,
            opt.origin.lat_deg, opt.origin.lon_deg,
            linked ? ", link " : "", linked ? opt.link.c_str() : "");
    }

    {
        FakeBase sim(opt, master);
        sim.run();
    }

    if (linked) ::unlink(opt.link.c_str());
    ::close(master);
    ::close(slave);
    return 0;
}
