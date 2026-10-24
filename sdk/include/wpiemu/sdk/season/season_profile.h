#pragma once

#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace wpiemu::sdk {

enum class SeasonChannel { Stable, Preview };

// A version pattern matches exactly, or as a prefix when it ends in '*'. A leading 'v' is ignored on both sides, so
// "v2026.3.4" matches "2026.3.4".
inline bool versionMatches(std::string_view pattern, std::string_view version) {
    if (!pattern.empty() && pattern.front() == 'v') pattern.remove_prefix(1);
    if (!version.empty() && version.front() == 'v') version.remove_prefix(1);
    if (!pattern.empty() && pattern.back() == '*') {
        pattern.remove_suffix(1);
        return version.substr(0, pattern.size()) == pattern;
    }
    return pattern == version;
}

inline bool versionVerified(const std::vector<std::string>& verified, std::string_view version) {
    for (const std::string& pattern : verified)
        if (versionMatches(pattern, version)) return true;
    return false;
}

// What the emulator knows about one vendor library on one season (for example the PhotonVision wire schema).
struct VendorProfile {
    std::string schema;                     // named wire format, e.g. "photon-us"
    std::string defaultVersion;             // assumed when the project does not declare a version
    std::vector<std::string> verified;      // versions the schema was verified against
};

// One WPILib season pack (seasons/<year>/season.json, docs/reference/seasons.md) as plain data. The build generates the
// built-in set from the manifests; core and the app read it and never name a year.
struct SeasonProfile {
    int year = 0;
    SeasonChannel channel = SeasonChannel::Stable;
    bool isDefault = false;
    std::string wpilibVersion;                       // the artifact version the extension is built against
    std::vector<std::string> verifiedWpilibVersions;
    std::string installFolder;                       // WPILib installer folder, e.g. "2026" or a preview suffix
    std::string halAbi;                              // seasons/abi/<id>.json
    std::string launchProfile;                       // "gradlerio-classic" | "gradlerio-modern"
    int javaMajor = 0;
    std::string gamePluginId;                        // empty: platform-only, core features only
    std::map<std::string, VendorProfile> vendors;    // keyed by vendor, e.g. "photonvision"

    bool verifiesWpilib(std::string_view version) const { return versionVerified(verifiedWpilibVersions, version); }
    const VendorProfile* vendor(const std::string& name) const {
        const auto it = vendors.find(name);
        return it == vendors.end() ? nullptr : &it->second;
    }
};

}  // namespace wpiemu::sdk
