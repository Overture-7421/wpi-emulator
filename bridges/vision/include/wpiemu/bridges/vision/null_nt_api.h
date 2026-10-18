#pragma once

// An INtApi that does nothing: the fallback while ntcore is not bound (or was not found), and a base for
// tests that only need the interface. Every call is the documented "unavailable" no-op.

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include "wpiemu/bridges/vision/nt_api.h"

namespace wpiemu::bridges::vision {

class NullNtApi final : public INtApi {
public:
    explicit NullNtApi(std::string_view reason = "ntcore is not available") : reason_(reason) {}

    bool available() const override { return false; }
    std::string_view unavailableReason() const override { return reason_; }
    std::int64_t nowUs() const override { return 0; }
    NtPublisher publish(std::string_view, NtType, const NtPublishOptions&) override { return {}; }
    bool setBoolean(NtPublisher, std::int64_t, bool) override { return false; }
    bool setInteger(NtPublisher, std::int64_t, std::int64_t) override { return false; }
    bool setDouble(NtPublisher, std::int64_t, double) override { return false; }
    bool setString(NtPublisher, std::int64_t, std::string_view) override { return false; }
    bool setRaw(NtPublisher, std::int64_t, std::span<const std::uint8_t>) override { return false; }
    bool setDoubleArray(NtPublisher, std::int64_t, std::span<const double>) override { return false; }
    bool setTopicProperty(std::string_view, std::string_view, std::string_view) override { return false; }
    NtSubscriber subscribe(std::string_view, std::size_t) override { return {}; }
    void read(NtSubscriber, std::vector<NtValue>&) override {}
    void topics(std::string_view, std::vector<NtTopicInfo>&) override {}
    bool hasForeignPublisher(std::string_view) override { return false; }

private:
    std::string_view reason_;  // must outlive the object: pass a literal
};

}  // namespace wpiemu::bridges::vision
