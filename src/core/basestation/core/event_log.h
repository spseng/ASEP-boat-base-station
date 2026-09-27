#pragma once

// Thread-safe, bounded list of human-readable events shown in the UI's
// event log ("boat 2 gate tripped", "sent Disable to all", ...).

#include <basestation/core/common.h>

#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace basestation {

enum class EventLevel : uint8_t { Info, Warn, Error };

struct Event {
    int64_t unix_us;
    EventLevel level;
    std::string text;
};

class EventLog {
public:
    explicit EventLog(size_t capacity = 2000) : capacity_(capacity) {}

    void add(EventLevel level, std::string text) {
        std::lock_guard<std::mutex> lock(mutex_);
        events_.push_back(Event{unix_time_us(), level, std::move(text)});
        while (events_.size() > capacity_) events_.pop_front();
        ++generation_;
    }
    void info(std::string text) { add(EventLevel::Info, std::move(text)); }
    void warn(std::string text) { add(EventLevel::Warn, std::move(text)); }
    void error(std::string text) { add(EventLevel::Error, std::move(text)); }

    std::vector<Event> snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return {events_.begin(), events_.end()};
    }
    // Incremented on every add; lets the UI skip copying when nothing changed.
    uint64_t generation() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return generation_;
    }
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        events_.clear();
        ++generation_;
    }

private:
    mutable std::mutex mutex_;
    std::deque<Event> events_;
    size_t capacity_;
    uint64_t generation_ = 0;
};

}  // namespace basestation
