#include "app.h"

#include <boat_defs/ids.h>
#include <boat_defs/serial.h>

#include <SDL3/SDL_filesystem.h>
#include <SDL3/SDL_stdinc.h>
#include <imgui_internal.h>  // DockBuilder API
#include <implot.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <sys/stat.h>

namespace basestation::app {

namespace {

// Panel window names. DockBuilder and the View menu refer to them by name.
constexpr const char* WIN_FLEET = "Fleet";
constexpr const char* WIN_BOATS = "Boats";
constexpr const char* WIN_COMMANDS = "Commands";
constexpr const char* WIN_TELEOP = "Teleop";
constexpr const char* WIN_PLOTS = "Plots";
constexpr const char* WIN_LINK = "Link";
constexpr const char* WIN_EVENTS = "Events";

constexpr double STALE_S = 3.0;
constexpr double BASE_FONT_PX = 16.0;

bool path_exists(const std::string& p) {
    struct stat st {};
    return !p.empty() && ::stat(p.c_str(), &st) == 0;
}

void copy_to(std::array<char, 512>& buf, const std::string& s) {
    std::snprintf(buf.data(), buf.size(), "%s", s.c_str());
}

}  // namespace

App::App(const Options& opts, float ui_scale, SDL_Renderer* renderer)
    : opts_(opts),
      renderer_(renderer),
      ui_scale_(ui_scale),
      link_(&events_, &frame_log_),
      fleet_(&events_),
      teleop_(link_, &events_),
      commander_(link_, &events_) {
    ImGuiIO& io = ImGui::GetIO();

    if (char* pref = SDL_GetPrefPath("spseng", "asep-base-station")) {
        pref_dir_ = pref;
        SDL_free(pref);
    } else {
        events_.warn(std::string("No settings directory, settings will not be saved: ") + SDL_GetError());
    }
    if (!pref_dir_.empty()) {
        settings_path_ = pref_dir_ + "settings.ini";
        imgui_ini_path_ = pref_dir_ + "imgui.ini";
    }

    settings_.baud = boat::serial::BAUD_RATE;
    if (!opts_.fresh && !settings_path_.empty()) {
        if (load_settings(settings_path_, settings_)) saved_settings_text_ = settings_to_string(settings_);
    }
    // --fresh (screenshots, tests) must not read or clobber the operator's layout.
    io.IniFilename = (opts_.fresh || imgui_ini_path_.empty()) ? nullptr : imgui_ini_path_.c_str();

    fonts_ = load_fonts(static_cast<float>(BASE_FONT_PX));
    apply_theme(ui_scale_);
    ImGui::GetStyle().FontScaleMain = settings_.text_scale;

    teleop_.set_config(settings_.teleop);
    apply_commander_config();
    fleet_.limits().history_seconds = 900.0;  // plot window max is 15 min

    // CLI options override the saved settings for this session only; they
    // are persisted only when the operator acts on them (e.g. connects).
    record_ = settings_.record && opts_.record;
    log_dir_ = opts_.log_dir.empty() ? settings_.log_dir : opts_.log_dir;
    baud_ = opts_.baud ? opts_.baud : settings_.baud;
    copy_to(port_buf_, opts_.port.empty() ? settings_.port : opts_.port);
    ports_ = list_serial_ports();
    if (port_buf_[0] == '\0' && !ports_.empty()) copy_to(port_buf_, ports_.front().path);

    // Map and base station. --map / --base act like choosing them in the UI.
    if (!opts_.map_url.empty()) {
        settings_.map.enabled = true;
        settings_.map.source = tiles::CUSTOM_SOURCE_ID;
        settings_.map.custom_url = opts_.map_url;
    }
    if (opts_.base_set) {
        settings_.base.manual_set = true;
        settings_.base.lat = opts_.base_lat;
        settings_.base.lon = opts_.base_lon;
    }
    copy_to(map_url_buf_, settings_.map.custom_url);
    open_map_layer();
    update_base_position();

    events_.info("ASEP base station started");
    if (opts_.select_boat > 0) select_boat(static_cast<uint8_t>(opts_.select_boat));
    if (opts_.connect) connect(opts_.port, baud_, true);
    if (opts_.test_teleop > 0) {
        set_teleop_target(static_cast<uint8_t>(opts_.test_teleop));
        set_teleop_enabled(true);
    }
    if (!opts_.focus_window.empty()) focus_frames_ = 4;
    rate_last_t_ = Clock::now();
}

App::~App() {
    if (!opts_.fresh) save_settings_now();
    // io.IniFilename points into imgui_ini_path_, which dies with us, but
    // ImGui::DestroyContext() (called later) would save through it: save
    // now and detach.
    ImGuiIO& io = ImGui::GetIO();
    if (io.IniFilename) {
        ImGui::SaveIniSettingsToDisk(io.IniFilename);
        io.IniFilename = nullptr;
    }
    // Members are destroyed in reverse order: commander_ and teleop_ first
    // (teleop sends its stop burst while link_ is still open), then link_,
    // then frame_log_.
}

void App::handle_event(const SDL_Event& event) {
    const std::optional<GamepadButton> pressed = gamepad_.handle_event(event, events_);
    if (!pressed) return;
    // The disable button acts on the edge, once per press, whether or not
    // teleop is enabled: it is a safety control.
    if (*pressed == settings_.teleop.disable_button) {
        if (teleop_target_ == 0) {
            events_.warn(std::string("Gamepad ") + to_string(*pressed) +
                         " pressed, but no teleop target is set: nothing sent");
        } else if (!link_.is_open()) {
            events_.error(std::string("Gamepad ") + to_string(*pressed) + ": not connected, DISABLE not sent");
        } else {
            commander_.disable(teleop_target_);
        }
    }
}

// ---------------------------------------------------------------------------
// Per-frame logic
// ---------------------------------------------------------------------------

void App::frame() {
    update_link();
    update_teleop();
    run_test_actions();
    update_map();

    draw_menu_bar();
    draw_toolbar();

    const ImGuiID dockspace_id = ImGui::GetID("MainDockSpace");
    if (reset_layout_ || ImGui::DockBuilderGetNode(dockspace_id) == nullptr) {
        build_default_layout(dockspace_id);
        reset_layout_ = false;
    }
    ImGui::DockSpaceOverViewport(dockspace_id, ImGui::GetMainViewport());

    draw_fleet_view();
    draw_boats_table();
    draw_commands();
    draw_teleop();
    draw_plots();
    draw_link();
    draw_events();
    draw_map_download_window();
    draw_confirm_popups();

    // Default tabs after a fresh layout, then any --focus request. Only the
    // last window focused in a frame gets its tab selected, so this is spread
    // over a few frames (docked windows exist only after their first Begin).
    if (focus_frames_ > 0) {
        --focus_frames_;
        if (focus_frames_ == 2 && default_tabs_pending_) ImGui::SetWindowFocus(WIN_BOATS);
        if (focus_frames_ == 1 && default_tabs_pending_) ImGui::SetWindowFocus(WIN_FLEET);
        if (focus_frames_ == 0) default_tabs_pending_ = false;
        if (focus_frames_ == 0 && !opts_.focus_window.empty()) ImGui::SetWindowFocus(opts_.focus_window.c_str());
    }

    update_settings_autosave();
}

void App::update_link() {
    const Clock::time_point now = Clock::now();

    rx_buf_.clear();
    link_.poll(rx_buf_);
    for (const RxFrame& rx : rx_buf_) fleet_.ingest(rx);

    // The local map frame is anchored at the base station if its position is
    // known, else at the first boat position we receive.
    update_base_position();
    if (!local_) {
        for (const auto& [id, b] : fleet_.boats()) {
            if (b.self_status && (b.self_status->self.lat != 0 || b.self_status->self.lon != 0)) {
                local_ = geo::LocalFrame(geo::from_e7(b.self_status->self.lat, b.self_status->self.lon));
                break;
            }
        }
    }

    // Frame rate over ~1 s windows (smooth enough to read, still responsive).
    const double dt = std::chrono::duration<double>(now - rate_last_t_).count();
    if (dt >= 1.0) {
        const uint64_t frames = link_.stats().rx_frames;
        rx_rate_ = frames >= rate_last_frames_ ? static_cast<double>(frames - rate_last_frames_) / dt : 0.0;
        rate_last_frames_ = frames;
        rate_last_t_ = now;
    }

    // Deferred disconnect: give teleop's stop burst time to go out first.
    if (pending_disconnect_at_ && now >= *pending_disconnect_at_) {
        pending_disconnect_at_.reset();
        link_.close();
        frame_log_.close();
    }

    if (!want_connected_ || link_.is_open()) return;

    // The link closed by itself (USB unplugged, simulator killed, ...).
    if (link_error_.empty()) {
        link_error_ = link_.last_error();
        if (link_error_.empty()) link_error_ = "link closed";
        if (frame_log_.is_open()) {
            events_.info("Session log closed: " + frame_log_.path() + " (" +
                         std::to_string(frame_log_.records()) + " records)");
            frame_log_.close();
        }
        connected_at_.reset();
        next_retry_ = now + std::chrono::seconds(1);
        retry_failure_logged_ = false;
        if (settings_.auto_reconnect)
            events_.warn("Will try to reconnect to " + conn_path_ + " every second");
    }
    if (settings_.auto_reconnect && now >= next_retry_) {
        next_retry_ = now + std::chrono::seconds(1);
        // Only try while the device node exists; opening a missing path
        // just produces an error every second.
        if (path_exists(conn_path_) && connect(conn_path_, conn_baud_, false))
            events_.info("Reconnected to " + conn_path_);
    }
}

void App::update_map() {
    map_->update();
    // Report the end of an area download in the event log.
    const tiles::TileService::PrefetchStatus pf = map_->service().prefetch_status();
    if (prefetch_was_active_ && !pf.active && !pf.message.empty()) {
        if (pf.failed) events_.warn("Map download: " + pf.message);
        else events_.info("Map download: " + pf.message);
    }
    prefetch_was_active_ = pf.active;
}

void App::update_teleop() {
    // Every frame, even with teleop off: the sender uses the snapshot's age
    // to detect a stalled UI.
    pad_ = gamepad_.snapshot();
    teleop_.update_input(pad_);
    teleop_status_ = teleop_.status();
    teleop_enabled_ = teleop_status_.enabled;
}

void App::run_test_actions() {
    if (opts_.test_disable > 0 && !test_disable_done_ && connected_at_ &&
        Clock::now() - *connected_at_ > std::chrono::seconds(3)) {
        test_disable_done_ = true;
        commander_.disable(static_cast<uint8_t>(opts_.test_disable));
    }
}

void App::update_settings_autosave() {
    if (opts_.fresh) return;
    const Clock::time_point now = Clock::now();
    // Keep the scalar view settings in sync with the live objects.
    settings_.text_scale = ImGui::GetStyle().FontScaleMain;
    const std::string text = settings_to_string(settings_);
    if (text == saved_settings_text_) {
        settings_dirty_ = false;
        return;
    }
    // Debounced: sliders change settings every frame while dragged.
    if (!settings_dirty_) {
        settings_dirty_ = true;
        settings_dirty_since_ = now;
    } else if (now - settings_dirty_since_ >= std::chrono::seconds(1)) {
        save_settings_now();
    }
}

void App::save_settings_now() {
    if (settings_path_.empty()) return;
    settings_.text_scale = ImGui::GetStyle().FontScaleMain;
    std::string err;
    if (save_settings(settings_path_, settings_, &err)) {
        saved_settings_text_ = settings_to_string(settings_);
    } else {
        events_.warn("Could not save settings: " + err);
        // Do not retry every frame; try again after the next change.
        saved_settings_text_ = settings_to_string(settings_);
    }
    settings_dirty_ = false;
}

// ---------------------------------------------------------------------------
// Connection
// ---------------------------------------------------------------------------

std::string App::effective_log_dir() const {
    if (!log_dir_.empty()) return log_dir_;
    return pref_dir_.empty() ? std::string("logs") : pref_dir_ + "logs";
}

bool App::connect(const std::string& path, uint32_t baud, bool user_initiated) {
    if (path.empty()) {
        events_.error("No serial port given");
        return false;
    }
    if (pending_disconnect_at_) {
        pending_disconnect_at_.reset();
        link_.close();
    }
    if (link_.is_open()) link_.close();
    frame_log_.close();

    // Open the log first so the very first received frame is recorded.
    std::string log_path;
    if (record_) {
        std::string err;
        std::string dir = effective_log_dir();
        if (!dir.empty() && dir.back() != '/') dir += '/';
        log_path = dir + default_log_filename();
        if (frame_log_.open(log_path, &err)) {
            events_.info("Recording session to " + log_path);
        } else {
            events_.error("Could not open session log: " + err);
            log_path.clear();
        }
    }

    std::string err;
    if (!link_.open(path, baud, &err)) {
        if (!log_path.empty()) {
            frame_log_.close();
            std::remove(log_path.c_str());  // header-only file, nothing recorded
        }
        if (user_initiated) {
            events_.error("Could not open " + path + ": " + err);
            want_connected_ = false;
            link_error_ = err;
        } else if (!retry_failure_logged_) {
            events_.warn("Reconnect to " + path + " failed: " + err + " (still retrying)");
            retry_failure_logged_ = true;
        }
        return false;
    }

    want_connected_ = true;
    conn_path_ = path;
    conn_baud_ = baud;
    link_error_.clear();
    connected_at_ = Clock::now();
    if (user_initiated) {
        settings_.port = path;
        settings_.baud = baud;
    }
    return true;
}

void App::disconnect() {
    want_connected_ = false;
    link_error_.clear();
    connected_at_.reset();
    if (!link_.is_open()) {
        frame_log_.close();
        return;
    }
    if (teleop_enabled_) {
        // Stop the boat before the link goes away: let the stop burst go out
        // (stop_burst commands at rate_hz), then close.
        set_teleop_enabled(false);
        const TeleopConfig& c = settings_.teleop;
        const double burst_s = (c.stop_burst + 1) / std::max(1.0f, c.rate_hz);
        pending_disconnect_at_ = Clock::now() + std::chrono::milliseconds(static_cast<int>(burst_s * 1000));
        return;
    }
    link_.close();
    if (frame_log_.is_open()) {
        events_.info("Session log closed: " + frame_log_.path() + " (" +
                     std::to_string(frame_log_.records()) + " records)");
    }
    frame_log_.close();
}

// ---------------------------------------------------------------------------
// Selection and targets
// ---------------------------------------------------------------------------

void App::select_boat(std::optional<uint8_t> id) {
    if (selected_ == id) return;
    selected_ = id;
    if (!id) return;
    // The command target follows the selection. So does the teleop target,
    // but never while driving: clicking a row must not move the joystick
    // to another boat.
    cmd_target_ = *id;
    if (!teleop_enabled_) teleop_target_ = *id;
}

void App::set_teleop_target(uint8_t id) {
    teleop_target_ = id;
    // The sender logs every change; only tell it while it matters.
    if (teleop_enabled_) teleop_.set_target(id);
}

void App::set_teleop_enabled(bool on) {
    if (on && teleop_target_ == 0) return;
    if (on) teleop_.set_target(teleop_target_);
    teleop_.set_enabled(on);
    teleop_enabled_ = on;
}

void App::apply_teleop_config(const TeleopConfig& cfg) {
    settings_.teleop = cfg;
    teleop_.set_config(cfg);
}

void App::apply_commander_config() {
    Commander::Config c;
    c.disable_repeats = std::max(1, settings_.disable_repeats);
    c.repeat_interval = std::chrono::milliseconds(std::max(10, settings_.repeat_interval_ms));
    commander_.set_config(c);
}

double App::age_s(const BoatState& b) const {
    return std::chrono::duration<double>(Clock::now() - b.last_heard).count();
}

bool App::is_stale(const BoatState& b) const { return age_s(b) > STALE_S; }

// ---------------------------------------------------------------------------
// Layout
// ---------------------------------------------------------------------------

void App::build_default_layout(ImGuiID id) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::DockBuilderRemoveNode(id);
    ImGui::DockBuilderAddNode(id, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(id, vp->WorkSize);

    // Fleet view large on the left, operator controls on the right, the
    // boats table under the map.
    ImGuiID right = 0, left = 0;
    ImGui::DockBuilderSplitNode(id, ImGuiDir_Right, 0.30f, &right, &left);
    ImGuiID left_bottom = 0, left_top = 0;
    ImGui::DockBuilderSplitNode(left, ImGuiDir_Down, 0.34f, &left_bottom, &left_top);
    ImGuiID right_bottom = 0, right_top = 0;
    ImGui::DockBuilderSplitNode(right, ImGuiDir_Down, 0.50f, &right_bottom, &right_top);

    ImGui::DockBuilderDockWindow(WIN_FLEET, left_top);
    ImGui::DockBuilderDockWindow(WIN_PLOTS, left_top);
    ImGui::DockBuilderDockWindow(WIN_LINK, left_top);
    ImGui::DockBuilderDockWindow(WIN_BOATS, left_bottom);
    ImGui::DockBuilderDockWindow(WIN_EVENTS, left_bottom);
    ImGui::DockBuilderDockWindow(WIN_COMMANDS, right_top);
    ImGui::DockBuilderDockWindow(WIN_TELEOP, right_bottom);
    ImGui::DockBuilderFinish(id);

    settings_.show_fleet = settings_.show_boats = settings_.show_commands = true;
    settings_.show_teleop = settings_.show_plots = settings_.show_link = settings_.show_events = true;
    focus_frames_ = 4;
    default_tabs_pending_ = true;
}

}  // namespace basestation::app
