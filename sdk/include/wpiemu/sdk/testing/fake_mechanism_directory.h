#pragma once

#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/mechanisms/attachment.h"

namespace wpiemu::sdk::testing {

class FakeMechanismDirectory final : public IMechanismDirectory {
public:
    explicit FakeMechanismDirectory(const RobotConfig& config, const MechanismFactory* factory = nullptr)
        : config_(config), factory_(factory) {}

    void add(IMechanism* mechanism) {
        if (mechanism != nullptr) mechanisms_[mechanism->id()] = mechanism;
    }

    IMechanism* find(std::string_view id) override {
        const auto it = mechanisms_.find(std::string(id));
        return it == mechanisms_.end() ? nullptr : it->second;
    }

    std::vector<IMechanism*> all() override {
        std::vector<IMechanism*> result;
        result.reserve(mechanisms_.size());
        for (const auto& [id, mechanism] : mechanisms_) result.push_back(mechanism);
        return result;
    }

    std::optional<Pose3> linkPoseInChassis(std::string_view mechanismId, std::string_view link) const override {
        return resolveLinkPoseInChassis(config_, mechanismId, link, [this](std::string_view id, std::string_view linkName) {
            const auto it = mechanisms_.find(std::string(id));
            if (it != mechanisms_.end()) {
                if (const auto pose = it->second->linkPose(linkName)) return pose;
            }
            if (linkName == kRootLink) return std::optional<Pose3>{Pose3{}};
            if (factory_ != nullptr) {
                const MechanismConfig* mechanism = config_.findMechanism(id);
                const MechanismTemplateInfo* info =
                    mechanism == nullptr ? nullptr : factory_->templateInfo(mechanism->type);
                if (info != nullptr)
                    for (const LinkSpec& spec : info->links)
                        if (spec.name == linkName) return std::optional<Pose3>{spec.local};
            }
            return std::optional<Pose3>{};
        });
    }

private:
    const RobotConfig& config_;
    const MechanismFactory* factory_;
    std::map<std::string, IMechanism*, std::less<>> mechanisms_;
};

}  // namespace wpiemu::sdk::testing
