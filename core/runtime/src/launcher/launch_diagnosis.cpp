#include "wpiemu/core/launcher/launch_diagnosis.h"

#include <array>
#include <cctype>

namespace wpiemu::core::launcher {

namespace {

std::string_view trim(std::string_view s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.remove_prefix(1);
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.remove_suffix(1);
    return s;
}

// The rest of the line after the first `key` found in `text`, trimmed; empty when absent.
std::string valueAfter(std::string_view text, std::string_view key) {
    const std::size_t at = text.find(key);
    if (at == std::string_view::npos) return {};
    std::string_view rest = text.substr(at + key.size());
    rest = rest.substr(0, rest.find_first_of("\r\n"));
    return std::string(trim(rest));
}

// The whole line holding the first occurrence of `needle`.
std::string lineWith(std::string_view text, std::string_view needle) {
    const std::size_t at = text.find(needle);
    if (at == std::string_view::npos) return {};
    const std::size_t begin = text.rfind('\n', at);
    const std::size_t end = text.find_first_of("\r\n", at);
    const std::size_t from = begin == std::string_view::npos ? 0 : begin + 1;
    return std::string(trim(text.substr(from, (end == std::string_view::npos ? text.size() : end) - from)));
}

bool containsAny(std::string_view text, std::initializer_list<std::string_view> needles, std::string* line = nullptr) {
    for (const std::string_view n : needles) {
        if (text.find(n) == std::string_view::npos) continue;
        if (line != nullptr) *line = lineWith(text, n);
        return true;
    }
    return false;
}

// "Expected at least 14.40, but found 14.36"
void readVersions(std::string_view text, LaunchProblem& p) {
    constexpr std::string_view kExpected = "Expected at least ";
    const std::size_t at = text.find(kExpected);
    if (at == std::string_view::npos) return;
    std::string_view rest = text.substr(at + kExpected.size());
    const auto number = [](std::string_view s, std::size_t& used) {
        used = 0;
        while (used < s.size() && (std::isdigit(static_cast<unsigned char>(s[used])) || s[used] == '.')) ++used;
        return std::string(s.substr(0, used));
    };
    std::size_t used = 0;
    p.expected = number(rest, used);
    const std::size_t found = rest.find("found ");
    if (found != std::string_view::npos) p.found = number(rest.substr(found + 6), used);
}

std::string extensionLoadFailure(std::string_view output) {
    bool emulatorExtension = false;
    while (!output.empty()) {
        const auto newline = output.find('\n');
        const auto line = trim(output.substr(0, newline));
        if (line.find("HAL Extensions: Attempting to load:") != std::string_view::npos)
            emulatorExtension = line.find("halsim_wpiemu") != std::string_view::npos;
        if (emulatorExtension &&
            (line.find("HAL Extensions: Failed to load library:") != std::string_view::npos ||
             line.find("HAL Extensions: Failed to load extension") != std::string_view::npos))
            return std::string(line);
        if (newline == std::string_view::npos) break;
        output.remove_prefix(newline + 1);
    }
    return {};
}

}  // namespace

LaunchProblem diagnoseLaunchOutput(std::string_view output) {
    LaunchProblem p;

    if (output.find("Invalid MSVC Runtime Detected") != std::string_view::npos || output.find("MsvcRuntimeException") != std::string_view::npos) {
        p.code = LaunchProblemCode::StaleVcRuntime;
        readVersions(output, p);
        p.jvmPath = valueAfter(output, "JVM Location:");
        p.runtimePath = valueAfter(output, "Runtime DLL Location:");
        p.detail = lineWith(output, "Expected at least");
        return p;
    }
    p.detail = extensionLoadFailure(output);
    if (!p.detail.empty()) {
        p.code = LaunchProblemCode::ExtensionLoadFailed;
        return p;
    }
    if (output.find("org.gradle.java.home") != std::string_view::npos && output.find("is invalid") != std::string_view::npos) {
        p.code = LaunchProblemCode::JavaHomeInvalid;
        p.detail = lineWith(output, "org.gradle.java.home");
        return p;
    }
    if (containsAny(output, {"JAVA_HOME is set to an invalid directory", "JAVA_HOME is not set and no 'java' command could be found"}, &p.detail)) {
        p.code = LaunchProblemCode::JavaHomeInvalid;
        return p;
    }
    if (containsAny(output, {"Could not determine java version from", "Unsupported class file major version", "Unsupported Java.",
                             "incompatible Java and Gradle versions", "Your build is currently configured to use incompatible Java"},
                    &p.detail)) {
        p.code = LaunchProblemCode::JavaUnsupportedByGradle;
        return p;
    }
    if (containsAny(output, {"Could not find or load main class org.gradle.wrapper.GradleWrapperMain", "GradleWrapperMain was not found"}, &p.detail)) {
        p.code = LaunchProblemCode::GradleWrapperBroken;
        return p;
    }
    if (containsAny(output, {"No tool chain is available to build", "No Toolchain Found for", "No toolchain found", "No tool chain is available"},
                    &p.detail)) {
        p.code = LaunchProblemCode::NoNativeToolchain;
        return p;
    }
    if (containsAny(output, {"Could not GET '", "Could not HEAD '", "UnknownHostException", "PKIX path building failed", "SSLHandshakeException",
                             "Network is unreachable", "Connect timed out", "Read timed out", "Connection timed out", "No such host is known",
                             "No cached version of", "Could not install Gradle distribution"},
                    &p.detail)) {
        p.code = LaunchProblemCode::DownloadFailed;
        return p;
    }
    return p;
}

const char* launchProblemId(LaunchProblemCode code) {
    switch (code) {
        case LaunchProblemCode::None: return "";
        case LaunchProblemCode::StaleVcRuntime: return "stale_vc_runtime";
        case LaunchProblemCode::JavaUnsupportedByGradle: return "java_unsupported_by_gradle";
        case LaunchProblemCode::JavaHomeInvalid: return "java_home_invalid";
        case LaunchProblemCode::DownloadFailed: return "download_failed";
        case LaunchProblemCode::GradleWrapperBroken: return "gradle_wrapper_broken";
        case LaunchProblemCode::NoNativeToolchain: return "no_native_toolchain";
        case LaunchProblemCode::ExtensionLoadFailed: return "extension_load_failed";
    }
    return "";
}

LaunchProblemCode launchProblemFromId(std::string_view id) {
    constexpr std::array<LaunchProblemCode, 7> all{LaunchProblemCode::StaleVcRuntime,      LaunchProblemCode::JavaUnsupportedByGradle,
                                                  LaunchProblemCode::JavaHomeInvalid,      LaunchProblemCode::DownloadFailed,
                                                  LaunchProblemCode::GradleWrapperBroken,  LaunchProblemCode::NoNativeToolchain,
                                                  LaunchProblemCode::ExtensionLoadFailed};
    for (const LaunchProblemCode c : all)
        if (id == launchProblemId(c)) return c;
    return LaunchProblemCode::None;
}

}  // namespace wpiemu::core::launcher
