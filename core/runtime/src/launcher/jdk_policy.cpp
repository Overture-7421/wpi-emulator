#include "wpiemu/core/launcher/jdk_policy.h"

#include <algorithm>
#include <cctype>
#include <system_error>
#include <utility>

namespace wpiemu::core::launcher {

namespace fs = std::filesystem;

namespace {

std::vector<long> components(std::string_view v) {
    std::vector<long> out;
    long current = 0;
    bool digits = false;
    for (const char c : v) {
        if (std::isdigit(static_cast<unsigned char>(c))) {
            current = current * 10 + (c - '0');
            digits = true;
        } else {
            if (c != '.') break;
            out.push_back(digits ? current : 0);
            current = 0;
            digits = false;
        }
    }
    if (digits) out.push_back(current);
    return out;
}

// Java major -> first Gradle that can run on it.
struct GradleRow {
    int java;
    std::string_view gradle;
};
constexpr GradleRow kGradleJava[] = {{8, "2.0"},  {11, "5.0"},  {17, "7.3"},  {20, "8.3"},  {21, "8.5"},
                                     {22, "8.8"}, {23, "8.10"}, {24, "8.14"}, {25, "9.1"}};

}  // namespace

int compareVersions(std::string_view a, std::string_view b) {
    const std::vector<long> x = components(a), y = components(b);
    for (std::size_t i = 0; i < std::max(x.size(), y.size()); ++i) {
        const long l = i < x.size() ? x[i] : 0, r = i < y.size() ? y[i] : 0;
        if (l != r) return l < r ? -1 : 1;
    }
    return 0;
}

int maxJavaForGradle(std::string_view gradleVersion, int seasonJavaMajor) {
    if (components(gradleVersion).empty()) gradleVersion = seasonJavaMajor >= 25 ? "9.1" : "8.11";
    int best = kGradleJava[0].java;
    for (const GradleRow& row : kGradleJava)
        if (compareVersions(gradleVersion, row.gradle) >= 0) best = row.java;
    return best;
}

std::optional<int> parseJavaMajor(std::string_view text) {
    const std::size_t at = text.find("version");
    if (at == std::string_view::npos) return std::nullopt;
    const std::size_t quote = text.find('"', at);
    if (quote == std::string_view::npos) return std::nullopt;
    const std::vector<long> parts = components(text.substr(quote + 1));
    if (parts.empty()) return std::nullopt;
    if (parts[0] == 1 && parts.size() > 1) return static_cast<int>(parts[1]);   // 1.8.0_352 -> 8
    return static_cast<int>(parts[0]);
}

JdkVerdict evaluateJdk(const JdkFacts& f, int minMajor, int maxMajor, bool windows) {
    JdkVerdict v;
    v.maxMajor = maxMajor;
    v.major = f.major.value_or(0);
    v.vcRuntime = f.vcRuntime.value_or("");
    if (!f.hasJava || !f.major) {
        v.problem = JdkProblem::NotRunnable;
        return v;
    }
    if (*f.major < minMajor) v.problem = JdkProblem::TooOld;
    else if (*f.major > maxMajor) v.problem = JdkProblem::TooNewForGradle;
    else if (!f.hasCompiler) v.problem = JdkProblem::NoCompiler;
    else if (windows && f.vcRuntime && compareVersions(*f.vcRuntime, kMinVcRuntime) < 0) v.problem = JdkProblem::StaleVcRuntime;
    return v;
}

fs::path managedToolchainsDir(const fs::path& localDataRoot) { return localDataRoot / "toolchains"; }

std::optional<fs::path> findManagedJdk(const fs::path& toolchainsDir, bool windows, int major) {
    std::error_code ec;
    std::vector<fs::path> ready;
    for (fs::directory_iterator it(toolchainsDir, ec), end; !ec && it != end; it.increment(ec)) {
        const fs::path& dir = it->path();
        if (!it->is_directory(ec)) continue;
        if (major > 0) {   // "temurin-17.0.20.1": the first number of the version is the Java major
            const std::string name = dir.filename().string();
            const std::size_t digits = name.find_first_of("0123456789");
            if (digits == std::string::npos || compareVersions(name.substr(digits, name.find('.', digits) - digits), std::to_string(major)) != 0) continue;
        }
        if (fs::is_regular_file(dir / std::string(kManagedJdkMarker), ec) && fs::is_regular_file(dir / "bin" / (windows ? "java.exe" : "java"), ec))
            ready.push_back(dir);
    }
    if (ready.empty()) return std::nullopt;
    // Names carry the version ("temurin-17.0.20.1"), so the newest sorts by its version part.
    std::sort(ready.begin(), ready.end(), [](const fs::path& a, const fs::path& b) {
        const std::string x = a.filename().string(), y = b.filename().string();
        const std::size_t ax = x.find_first_of("0123456789"), ay = y.find_first_of("0123456789");
        return compareVersions(ax == std::string::npos ? "" : x.substr(ax), ay == std::string::npos ? "" : y.substr(ay)) > 0;
    });
    return ready.front();
}

}  // namespace wpiemu::core::launcher
