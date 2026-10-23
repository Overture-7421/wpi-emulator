#include <photon/PhotonCamera.h>
#include <wpi/nt/ntcore_c.h>

#include <cstdio>
#include <optional>
#include <string>
#include <string_view>
#include <wpi/framework/RobotBase.hpp>
#include <wpi/framework/TimedRobot.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/nt/RawTopic.hpp>
#include <wpi/system/Timer.hpp>
#include <wpi/util/json.hpp>

int probeUuidConversion() {
    const std::string uuid = "fd353b11f3a818e9c5b457c7dc5a8f5c";
    auto instance = wpi::nt::NetworkTableInstance::Create();
    auto topic = instance.GetRawTopic("/uuid-probe");
    auto publisher = topic.Publish("raw");
    const std::string jsonText = "\"" + uuid + "\"";
    const WPI_String name{"message_uuid", 12};
    const WPI_String value{jsonText.data(), jsonText.size()};
    const bool published = NT_SetTopicProperty(topic.GetHandle(), &name, &value) != 0;
    const auto property = topic.GetProperty("message_uuid");
    const std::string converted{property};
    const bool decodedMatches = property.is_string() && property.get_string() == uuid;
    std::printf(
        "UUID_PROBE published=%d isString=%d implicit=[%s] implicitMatches=%d decodedMatches=%d\n",
        published ? 1 : 0, property.is_string() ? 1 : 0, converted.c_str(),
        converted == uuid ? 1 : 0, decodedMatches ? 1 : 0);
    wpi::nt::NetworkTableInstance::Destroy(instance);
    return published && decodedMatches ? 0 : 1;
}

class Robot final : public wpi::TimedRobot {
public:
    void RobotPeriodic() override {
        const double now = wpi::Timer::GetMonotonicTimestamp().value();
        if (!startedAt_) startedAt_ = now;
        const double elapsed = now - *startedAt_;
        photon_.SetDriverMode(elapsed >= 1.0 && elapsed < 1.5);
        photon_.SetEnabled(!(elapsed >= 2.0 && elapsed < 2.5));
        photon_.SetPipelineIndex(elapsed >= 2.5 ? 2 : 0);
        for (auto& next : photon_.GetAllUnreadResults()) result_ = std::move(next);
        if (now - lastPrint_ < 0.2) return;
        lastPrint_ = now;
        const auto pose = limelight_->GetEntry("botpose_wpiblue").GetDoubleArray({});
        const auto& multi = result_.MultiTagResult();
        const auto target =
            result_.HasTargets() ? std::optional{result_.GetBestTarget()} : std::nullopt;
        std::printf(
            "VISION2027 ll=%d llTags=%.0f llX=%.4f llY=%.4f llLatency=%.1f llAge=%.2f"
            " photon=%d tags=%zu multi=%d x=%.4f y=%.4f age=%.2f sequence=%lld pipeline=%d "
            "driver=%d enabled=%d"
            " latency=%.2f id=%d targetX=%.4f corners=%zu calibration=%d\n",
            limelight_->GetEntry("tv").GetDouble(0) == 1 ? 1 : 0, pose.size() > 7 ? pose[7] : 0,
            pose.size() > 0 ? pose[0] : 0, pose.size() > 1 ? pose[1] : 0,
            limelight_->GetEntry("tl").GetDouble(0) + limelight_->GetEntry("cl").GetDouble(0),
            (now - limelight_->GetEntry("hb").GetLastChange() / 1e9) * 1000,
            photon_.IsConnected() ? 1 : 0, result_.GetTargets().size(), multi.has_value() ? 1 : 0,
            multi ? multi->estimatedPose.best.X().value() : 0,
            multi ? multi->estimatedPose.best.Y().value() : 0,
            (now - result_.GetTimestamp().value()) * 1000,
            static_cast<long long>(result_.SequenceID()), photon_.GetPipelineIndex(),
            photon_.GetDriverMode() ? 1 : 0, photon_.GetEnabled() ? 1 : 0,
            result_.GetLatency().value(), target ? target->GetFiducialId() : -1,
            target ? target->GetBestCameraToTarget().X().value() : 0,
            target ? target->GetDetectedCorners().size() : std::size_t{0},
            photon_.GetCameraMatrix().has_value() && photon_.GetDistCoeffs().has_value() ? 1 : 0);
        std::fflush(stdout);
    }

private:
    photon::PhotonCamera photon_{"front"};
    std::shared_ptr<wpi::nt::NetworkTable> limelight_ =
        wpi::nt::NetworkTableInstance::GetDefault().GetTable("limelight-front");
    photon::PhotonPipelineResult result_;
    std::optional<double> startedAt_;
    double lastPrint_ = 0;
};

int main(int argc, char** argv) {
    if (argc == 2 && std::string_view(argv[1]) == "--uuid-probe") return probeUuidConversion();
    return wpi::StartRobot<Robot>();
}
