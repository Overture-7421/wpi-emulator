#pragma once

// IMechanismDirectory over a plain list of mechanisms, with the SDK 1.1 resolved link transforms
// (mechanisms/attachment.h). Lets plugin/UI tests exercise parent links without the real runtime.
//
//   FakeMechanismDirectory dir(env.config);        // attachments come from the config
//   dir.add(&elevator); dir.add(&arm);
//   env.ctx.bindMechanisms(&dir);
//   dir.linkPoseInChassis("arm", "root");

#include <algorithm>
#include <vector>

#include "wpiemu/sdk/mechanisms/attachment.h"

namespace wpiemu::sdk::testing {

class FakeMechanismDirectory final : public IMechanismDirectory {
public:
    // `config` (attachments, mounts) and `factory` (optional: static link rest poses) must outlive the directory.
    explicit FakeMechanismDirectory(const RobotConfig& config, const MechanismFactory* factory = nullptr)
        : config_(&config), factory_(factory) {}

    void add(IMechanism* mechanism) { mechanisms_.push_back(mechanism); }

    IMechanism* find(std::string_view id) override { return lookup(id); }
    std::vector<IMechanism*> all() override { return mechanisms_; }

    std::optional<Pose3> linkPoseInChassis(std::string_view mechanismId, std::string_view link) const override {
        return resolveLinkPoseInChassis(*config_, mechanismId, link,
                                        [this](std::string_view id, std::string_view l) { return linkInRoot(id, l); });
    }

private:
    IMechanism* lookup(std::string_view id) const {
        for (IMechanism* m : mechanisms_)
            if (m->id() == id) return m;
        return nullptr;
    }
    std::optional<Pose3> linkInRoot(std::string_view id, std::string_view link) const {
        if (link == kRootLink) return Pose3{};
        if (const IMechanism* m = lookup(id))
            if (auto p = m->linkPose(link)) return p;
        // not published yet: fall back to the template's rest pose
        const MechanismConfig* cfg = config_->findMechanism(id);
        if (cfg != nullptr && factory_ != nullptr)
            if (const MechanismTemplateInfo* info = factory_->templateInfo(cfg->type))
                for (const LinkSpec& l : info->links)
                    if (l.name == link) return l.local;
        return std::nullopt;
    }

    const RobotConfig* config_;
    const MechanismFactory* factory_;
    std::vector<IMechanism*> mechanisms_;
};

}  // namespace wpiemu::sdk::testing
