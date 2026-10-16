#include <basestation/core/uplink.h>

#include <algorithm>
#include <cstring>

namespace basestation {

namespace {

bool same_frame(const wirelink::Frame& a, const wirelink::Frame& b) {
    return a.type == b.type && a.len == b.len && std::memcmp(a.payload.data.data(), b.payload.data.data(), a.len) == 0;
}

bool is_disable(const wirelink::Frame& f) { return f.type == static_cast<uint8_t>(lora::MsgType::Disable); }

}  // namespace

bool is_land_type(uint8_t type) {
    switch (static_cast<lora::MsgType>(type)) {
    case lora::MsgType::Disable:
    case lora::MsgType::Command:
    case lora::MsgType::Heartbeat:
    case lora::MsgType::Reenable:
    case lora::MsgType::SetMode: return true;
    case lora::MsgType::SelfStatus: return false;
    }
    return false;
}

Uplink::Uplink(LinkSession& link, EventLog* events) : link_(link), events_(events) {
    link_.set_rx_listener([this](const RxFrame& rx) { on_frame(rx); });
    thread_ = std::thread([this] { loop(); });
}

Uplink::~Uplink() {
    // Unregister first: waits for a callback in progress, which takes mutex_.
    link_.set_rx_listener(nullptr);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        quit_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

void Uplink::set_config(const Config& cfg) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cfg_ = cfg;
    }
    cv_.notify_all();
}

Uplink::Config Uplink::config() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cfg_;
}

Uplink::Status Uplink::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Status s = status_;
    s.queued = queue_.size();
    if (!periods_.empty()) {
        const auto [lo, hi] = std::minmax_element(periods_.begin(), periods_.end());
        s.period_min_s = *lo;
        s.period_max_s = *hi;
    }
    return s;
}

void Uplink::reset_stats() {
    std::lock_guard<std::mutex> lock(mutex_);
    status_ = Status{};
    periods_.clear();
}

bool Uplink::push(const wirelink::Frame& f, int copies, const std::string& label) {
    const auto now = Clock::now();
    bool open = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        Item it{};
        it.frame = f;
        it.label = label;
        it.copies = std::max(1, copies);
        it.queued = now;

        auto dup = std::find_if(queue_.begin(), queue_.end(), [&](const Item& q) { return same_frame(q.frame, f); });
        if (dup != queue_.end() && is_disable(f)) {
            // A second press for the same target restarts its copies rather
            // than queueing them twice.
            dup->label = label;
            dup->copies = it.copies;
            dup->confirmed = dup->unconfirmed = 0;
            dup->queued = now;
        } else if (is_disable(f)) {
            // Ahead of everything but other Disables: stopping a boat must
            // not wait behind a mode change.
            auto pos = std::find_if(queue_.begin(), queue_.end(), [](const Item& q) { return !is_disable(q.frame); });
            if (pos == queue_.begin() && pos != queue_.end()) {
                pos->in_mailbox = false;  // about to be overwritten in the mailbox
                pos->attempts = 0;
            }
            queue_.insert(pos, it);
        } else {
            queue_.push_back(it);
        }
        hand_head(now);
        open = link_.is_open();
    }
    cv_.notify_all();
    return open;
}

void Uplink::drop_if(const std::function<bool(const wirelink::Frame&)>& pred) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.erase(std::remove_if(queue_.begin(), queue_.end(), [&](const Item& it) { return pred(it.frame); }),
                 queue_.end());
    // The new head (if any) replaces whatever the mailbox holds.
    hand_head(Clock::now());
}

Uplink::Offer Uplink::offer(const lora::Command& cmd) {
    wirelink::Frame f{};
    if (!wirelink::codec::pack(cmd, f)) return Offer::Failed;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!queue_.empty()) {
        ++status_.commands_held;
        return Offer::Held;
    }
    // Under the lock, so a one-shot frame pushed meanwhile is always handed
    // over after this Command and wins the mailbox.
    return link_.send(f) ? Offer::Sent : Offer::Failed;
}

void Uplink::hand_head(Clock::time_point now) {
    if (queue_.empty()) return;
    Item& h = queue_.front();
    if (h.in_mailbox) return;
    h.handed = now;  // also paces retries while the link is closed
    if (link_.send(h.frame)) h.in_mailbox = true;
}

void Uplink::finish_copy(Item& it, bool confirmed) {
    if (confirmed) {
        ++it.confirmed;
    } else {
        ++it.unconfirmed;
        ++status_.unconfirmed;
    }
    it.attempts = 0;
    it.in_mailbox = false;
}

void Uplink::pop_finished() {
    for (auto it = queue_.begin(); it != queue_.end();) {
        if (it->confirmed + it->unconfirmed < it->copies) {
            ++it;
            continue;
        }
        if (it->unconfirmed > 0 && events_) {
            events_->warn(it->label + ": " + std::to_string(it->unconfirmed) + " of " + std::to_string(it->copies) +
                          (it->copies == 1 ? " copy" : " copies") + " not confirmed by the base station");
        }
        it = queue_.erase(it);
    }
}

void Uplink::on_frame(const RxFrame& rx) {
    const wirelink::Frame& f = rx.frame;
    if (!is_land_type(f.type)) return;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // An echo: the ESP32 has just transmitted its land slot.
        if (status_.slots > 0) {
            const double dt = std::chrono::duration<double>(rx.t - status_.last_slot).count();
            // Short gaps too (back-to-back slots are worth seeing); long
            // ones are the link pausing, not the schedule.
            if (dt < 1.0) {
                periods_.push_back(dt);
                if (periods_.size() > PERIOD_WINDOW) periods_.pop_front();
            }
            if (dt > 0.02 && dt < 1.0)
                status_.slot_period_s = std::isnan(status_.slot_period_s) ? dt : 0.8 * status_.slot_period_s + 0.2 * dt;
        }
        ++status_.slots;
        status_.last_slot = rx.t;

        bool head_matched = false;
        bool ours = false;
        for (size_t i = 0; i < queue_.size(); ++i) {
            Item& it = queue_[i];
            if (it.handed == Clock::time_point{} || !same_frame(it.frame, f)) continue;
            ours = true;
            head_matched = i == 0;
            ++status_.oneshots_on_air;
            finish_copy(it, true);
            break;
        }
        if (!ours) {
            if (f.type == static_cast<uint8_t>(lora::MsgType::Heartbeat))
                ++status_.heartbeats;
            else if (f.type == static_cast<uint8_t>(lora::MsgType::Command))
                ++status_.commands_on_air;
        }

        // The head was in the mailbox but this slot carried something else:
        // it was overwritten or lost on the way. Hand it again, a few times.
        if (!head_matched && !queue_.empty() && queue_.front().in_mailbox) {
            Item& h = queue_.front();
            if (++h.attempts >= cfg_.max_attempts)
                finish_copy(h, false);
            else
                h.in_mailbox = false;
        }
        pop_finished();
        hand_head(rx.t);
    }
    cv_.notify_all();
}

void Uplink::loop() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!quit_) {
        if (queue_.empty()) {
            cv_.wait(lock, [this] { return quit_ || !queue_.empty(); });
            continue;
        }
        // Next thing to do: the head's echo timeout (or its retry while the
        // link is closed), or an item reaching max_age.
        auto due = queue_.front().handed + cfg_.echo_timeout;
        for (const Item& it : queue_) due = std::min(due, it.queued + cfg_.max_age);
        if (Clock::now() < due) {
            cv_.wait_until(lock, due);
            continue;
        }
        const auto now = Clock::now();

        for (auto it = queue_.begin(); it != queue_.end();) {
            if (now - it->queued < cfg_.max_age) {
                ++it;
                continue;
            }
            const int done = it->confirmed + it->unconfirmed;
            ++status_.dropped;
            if (events_) {
                if (done == 0)
                    events_->error(it->label + ": not sent (link closed?)");
                else
                    events_->warn(it->label + ": only " + std::to_string(done) + " of " + std::to_string(it->copies) +
                                  " copies sent");
            }
            it = queue_.erase(it);
        }
        if (!queue_.empty()) {
            Item& h = queue_.front();
            if (h.in_mailbox && now - h.handed >= cfg_.echo_timeout) {
                // No echo at all within the timeout: the firmware does not
                // echo, or USB is stalled. Count the copy and move on.
                finish_copy(h, false);
                pop_finished();
            }
        }
        hand_head(now);
    }
}

}  // namespace basestation
