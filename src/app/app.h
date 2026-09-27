#pragma once

// The base-station UI. App owns every core object and draws all panels;
// the panels are split across ui_*.cpp files but share App's state (the
// selected boat, targets, the link) so they stay consistent.

#include "cli.h"
#include "gamepad.h"
#include "settings.h"
#include "theme.h"

#include <basestation/core/commander.h>
#include <basestation/core/event_log.h>
#include <basestation/core/fleet_model.h>
#include <basestation/core/frame_log.h>
#include <basestation/core/geo.h>
#include <basestation/core/link_session.h>
#include <basestation/core/serial_port.h>
#include <basestation/core/teleop.h>
#include <basestation/core/teleop_sender.h>
#include <basestation/proto/names.h>

#include <SDL3/SDL_events.h>
#include <imgui.h>

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace basestation::app {

class App {
public:
    App(const Options& opts, float ui_scale);
    ~App();
    App(const App&) = delete;
    App& operator=(const App&) = delete;

    void handle_event(const SDL_Event& event);
    void frame();  // builds the whole UI for one frame
    bool quit_requested() const { return quit_; }

private:
    // ---- per-frame logic (app.cpp) ----
    void update_link();
    void update_teleop();
    void update_settings_autosave();
    void run_test_actions();
    void save_settings_now();

    // ---- connection (app.cpp) ----
    bool connect(const std::string& path, uint32_t baud, bool user_initiated);
    void disconnect();
    std::string effective_log_dir() const;

    // ---- selection / targets (app.cpp) ----
    void select_boat(std::optional<uint8_t> id);
    uint8_t teleop_target() const { return teleop_target_; }
    void set_teleop_enabled(bool on);
    void set_teleop_target(uint8_t id);
    void apply_teleop_config(const TeleopConfig& cfg);
    void apply_commander_config();
    // Seconds since the boat was last heard (any frame).
    double age_s(const BoatState& b) const;
    bool is_stale(const BoatState& b) const;

    // ---- layout (app.cpp) ----
    void build_default_layout(ImGuiID dockspace_id);

    // ---- panels ----
    void draw_menu_bar();       // ui_toolbar.cpp
    void draw_toolbar();        // ui_toolbar.cpp
    void draw_fleet_view();     // ui_fleet_view.cpp
    void draw_boats_table();    // ui_boats.cpp
    void draw_commands();       // ui_commands.cpp
    void draw_boat_detail(const BoatState& b);  // ui_commands.cpp
    void draw_teleop();         // ui_teleop.cpp
    void draw_teleop_config();  // ui_teleop.cpp
    void draw_plots();          // ui_plots.cpp
    void draw_link();           // ui_link.cpp
    void draw_events();         // ui_events.cpp
    void draw_confirm_popups(); // ui_commands.cpp (non-modal on purpose)

    // Target combo used by Commands and Teleop: boats seen + ALL.
    bool target_combo(const char* label, uint8_t& target, bool allow_all);

    // ---- configuration ----
    Options opts_;
    float ui_scale_;
    bool quit_ = false;
    std::string pref_dir_;
    std::string settings_path_;
    std::string imgui_ini_path_;  // io.IniFilename points into this
    Settings settings_;
    std::string saved_settings_text_;  // what is on disk, to detect changes
    bool settings_dirty_ = false;
    Clock::time_point settings_dirty_since_{};
    Fonts fonts_;

    // ---- core objects. Declaration order = construction order; the sender
    // and commander hold a reference to link_, so they are declared after it
    // and therefore destroyed before it (teleop's stop burst still goes out).
    EventLog events_;
    FrameLogWriter frame_log_;
    LinkSession link_;
    FleetModel fleet_;
    TeleopSender teleop_;
    Commander commander_;
    Gamepad gamepad_;
    GamepadSnapshot pad_{};

    // ---- connection state ----
    std::array<char, 512> port_buf_{};
    uint32_t baud_ = 0;
    std::vector<SerialPortInfo> ports_;
    bool record_ = true;  // record a log on the next connect
    std::string log_dir_;  // session value (CLI or settings)
    std::array<char, 512> log_dir_buf_{};  // Link panel editor
    bool want_connected_ = false;  // user asked to be connected
    std::string conn_path_;        // path we (re)connect to
    uint32_t conn_baud_ = 0;
    std::string link_error_;       // why the link dropped (shown in the toolbar)
    Clock::time_point next_retry_{};
    bool retry_failure_logged_ = false;
    std::optional<Clock::time_point> pending_disconnect_at_;  // after teleop stop burst
    std::vector<RxFrame> rx_buf_;
    // Received-frames-per-second, measured over ~1 s windows.
    uint64_t rate_last_frames_ = 0;
    Clock::time_point rate_last_t_{};
    double rx_rate_ = 0;

    // ---- selection and targets ----
    std::optional<uint8_t> selected_;
    uint8_t cmd_target_ = 0;         // 0 = none yet
    uint8_t teleop_target_ = 0;      // 0 = none yet
    bool teleop_enabled_ = false;    // mirrors TeleopSender, set from the UI
    TeleopSender::Status teleop_status_{};

    // Commands panel.
    int cmd_mode_idx_ = 0;           // index into names::MODES, or MODES size = raw
    int cmd_mode_raw_ = 0;           // "Other (raw value)"
    bool cmd_armed_ = false;
    bool confirm_reenable_open_ = false;  // confirmation dialogs (non-modal)
    bool confirm_arm_open_ = false;
    bool confirm_appearing_ = false;
    uint8_t popup_target_ = 0;
    uint8_t popup_mode_ = 0;
    bool popup_armed_ = false;

    // ---- fleet view ----
    std::optional<geo::LocalFrame> local_;
    bool map_auto_fit_ = true;
    bool map_fit_once_ = false;
    bool map_recenter_ = false;
    bool map_show_trails_ = true;
    bool map_show_labels_ = true;
    double map_span_x_ = 120;  // current view size, for Re-center
    double map_span_y_ = 120;

    // ---- plots ----
    bool plot_follow_ = true;
    double plot_x_min_ = 0;
    double plot_x_max_ = 120;

    // ---- events ----
    ImGuiTextFilter events_filter_;
    bool events_autoscroll_ = true;
    uint64_t events_gen_ = ~0ull;
    std::vector<Event> events_cache_;

    // ---- layout ----
    bool reset_layout_ = false;
    int focus_frames_ = 0;  // countdown to selecting default tabs / --focus
    bool default_tabs_pending_ = false;
    Clock::time_point start_t_ = Clock::now();
    std::optional<Clock::time_point> connected_at_;
    bool test_disable_done_ = false;
};

// ---- helpers shared by the panels (ui_util.cpp) ----
std::string boat_label(uint8_t id);      // "B2" / "ALL"
const char* msg_type_name(uint8_t type);  // lora:: / base:: message type name
ImVec4 age_color(double age_s);          // green / amber / red by age
// Wall-clock microseconds -> "HH:MM:SS.mmm" (local time).
std::string format_clock(int64_t unix_us);
std::string format_duration(double seconds);  // "3.2 s", "4 min 10 s", "2 h 05 min"
void help_marker(const char* text);
// Coloured, framed status chip: filled rounded rect with centred text.
void status_chip(const char* text, const ImVec4& color, float min_width = 0.0f);

}  // namespace basestation::app
