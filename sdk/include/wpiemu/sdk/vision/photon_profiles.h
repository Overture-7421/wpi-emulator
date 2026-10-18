#pragma once

// PhotonVision wire profiles, shared by the project inspector (which knows the project's photonlib
// version) and the PhotonVision bridge (which knows which message layouts it can write). A
// PhotonVision result message is identified by the UUID in its NetworkTables type string
// (`photonstruct:PhotonPipelineResult:<uuid>`): it is the MD5 of the schema, and PhotonLib throws
// when it differs from its own, so the bridge never guesses a layout (ADR 0017).

#include <cstddef>
#include <string>
#include <string_view>

namespace wpiemu::sdk {

// Message UUID of PhotonLib v2025.3.2 .. v2026.3.4 (verified identical in v2025.3.2, v2026.1.1 and
// v2026.3.4); the version rule below also accepts v2025.3.1.
inline constexpr const char* kPhotonMessageUuidV2025_3 = "4b2ff16a964b5e2bf04be0c1454d91c4";

// The message UUID a PhotonLib release speaks, or "" when it is unknown or unverified: newer than
// the verified range, another year line, or not parsable. Accepts "v2026.3.4" and "2026.3.4" (three
// numeric components, nothing else). Rule: year 2025 with version >= 2025.3.1, or year 2026 with
// version <= 2026.3.4.
inline std::string photonMessageUuidForVersion(std::string_view photonlibVersion) {
    std::string_view rest = photonlibVersion;
    if (!rest.empty() && (rest.front() == 'v' || rest.front() == 'V')) rest.remove_prefix(1);

    int parts[3] = {0, 0, 0};
    for (int i = 0; i < 3; ++i) {
        std::size_t digits = 0;
        int value = 0;
        while (digits < rest.size() && rest[digits] >= '0' && rest[digits] <= '9') {
            if (digits == 6) return {};  // not a plausible version component
            value = value * 10 + (rest[digits] - '0');
            ++digits;
        }
        if (digits == 0) return {};
        parts[i] = value;
        rest.remove_prefix(digits);
        if (i < 2) {
            if (rest.empty() || rest.front() != '.') return {};
            rest.remove_prefix(1);
        }
    }
    if (!rest.empty()) return {};

    const int year = parts[0];
    const int minor = parts[1];
    const int patch = parts[2];
    if (year == 2025 && (minor > 3 || (minor == 3 && patch >= 1))) return kPhotonMessageUuidV2025_3;
    if (year == 2026 && (minor < 3 || (minor == 3 && patch <= 4))) return kPhotonMessageUuidV2025_3;
    return {};
}

}  // namespace wpiemu::sdk
