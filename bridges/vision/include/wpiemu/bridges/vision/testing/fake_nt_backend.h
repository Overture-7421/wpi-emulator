#pragma once

// In-memory INtBackend for unit tests: records every publication (type, type string, history with explicit
// timestamps) and lets tests inject robot-written values.

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/bridges/vision/nt_backend.h"

namespace wpiemu::bridges::vision::testing {

class FakeNtBackend final : public INtBackend {
public:
    struct Sample {
        NtValue value;
        std::int64_t timestampUs = 0;
    };
    struct Topic {
        std::string typeString;
        std::vector<Sample> history;
        const Sample& last() const { return history.back(); }
    };

    bool available() override { return isAvailable; }
    std::int64_t nowUs() override { return nowTimeUs; }

    sdk::Status publish(std::string_view path, const NtValue& value, std::int64_t timestampUs,
                        std::string_view typeString = {}) override {
        if (!isAvailable) return sdk::makeError(sdk::ErrorCode::InvalidState, "ntcore is not available");
        Topic& t = topics[std::string(path)];
        if (!t.history.empty() && t.history.back().value.index() != value.index()) {
            return sdk::makeError(sdk::ErrorCode::InvalidArgument, "topic type changed: " + std::string(path));
        }
        if (t.history.empty()) t.typeString = std::string(typeString);
        t.history.push_back({value, timestampUs});
        ++publishCount;
        return sdk::okStatus();
    }

    std::optional<NtValue> read(std::string_view path) override {
        const auto it = robotValues.find(std::string(path));
        if (it == robotValues.end()) return std::nullopt;
        return it->second;
    }

    void unpublish(std::string_view pathPrefix) override {
        for (auto it = topics.begin(); it != topics.end();) {
            if (it->first.compare(0, pathPrefix.size(), pathPrefix) == 0) {
                it = topics.erase(it);
            } else {
                ++it;
            }
        }
    }

    // ---- test helpers -------------------------------------------------------------------------------------
    void setRobotValue(const std::string& path, NtValue value) { robotValues[path] = std::move(value); }
    bool has(const std::string& path) const { return topics.count(path) != 0; }
    const Topic* topic(const std::string& path) const {
        const auto it = topics.find(path);
        return it == topics.end() ? nullptr : &it->second;
    }
    template <class T>
    const T* lastAs(const std::string& path) const {
        const Topic* t = topic(path);
        return t == nullptr || t->history.empty() ? nullptr : std::get_if<T>(&t->last().value);
    }
    double lastDouble(const std::string& path, double fallback = -1e300) const {
        const double* v = lastAs<double>(path);
        return v != nullptr ? *v : fallback;
    }
    const NtDoubles& lastDoubles(const std::string& path) const {
        static const NtDoubles kEmpty;
        const NtDoubles* v = lastAs<NtDoubles>(path);
        return v != nullptr ? *v : kEmpty;
    }

    bool isAvailable = true;
    std::int64_t nowTimeUs = 0;
    std::map<std::string, Topic> topics;
    std::map<std::string, NtValue> robotValues;
    std::size_t publishCount = 0;
};

}  // namespace wpiemu::bridges::vision::testing
