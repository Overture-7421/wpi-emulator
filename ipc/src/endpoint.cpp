#include "wpiemu/ipc/endpoint.h"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <random>

#include "platform/platform_ipc.h"

namespace wpiemu::ipc {

using namespace sdk;

namespace {

std::uint64_t splitmix64(std::uint64_t x) {
    x += 0x9E3779B97F4A7C15ULL;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31);
}

// Unguessable and unique per call even if random_device is weak (some VMs return constants): the
// OS entropy is mixed with the clock and a process-wide counter.
std::string randomToken() {
    static constexpr char kHex[] = "0123456789abcdef";
    static std::atomic<std::uint64_t> counter{0};
    std::random_device rd;
    std::uint64_t mix = (static_cast<std::uint64_t>(rd()) << 32) ^ rd();
    mix = splitmix64(mix ^ static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count()));
    mix = splitmix64(mix + counter.fetch_add(1) * 0x9E3779B97F4A7C15ULL);
    std::string token;
    for (int i = 0; i < 16; ++i) {
        token.push_back(kHex[mix & 0xF]);
        mix >>= 4;
    }
    return token;
}

bool isNameChar(char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.';
}

bool isPlainName(std::string_view s) {
    if (s.empty() || s.size() > 200) return false;
    for (char c : s)
        if (!isNameChar(c)) return false;
    return true;
}

}  // namespace

ConnectionEndpoint ConnectionEndpoint::generate() {
    return ConnectionEndpoint(platform::nativeAddressFor(randomToken()), platform::nativeKind());
}

ConnectionEndpoint ConnectionEndpoint::generateInProcess() {
    return ConnectionEndpoint(std::string(kInProcessPrefix) + randomToken(), EndpointKind::InProcess);
}

Result<ConnectionEndpoint> ConnectionEndpoint::parse(std::string address) {
    if (address.compare(0, kNamedPipePrefix.size(), kNamedPipePrefix) == 0) {
        if (!isPlainName(std::string_view(address).substr(kNamedPipePrefix.size())))
            return makeError(ErrorCode::InvalidArgument, "invalid named-pipe endpoint: " + address);
        return ConnectionEndpoint(std::move(address), EndpointKind::NamedPipe);
    }
    if (address.compare(0, kInProcessPrefix.size(), kInProcessPrefix) == 0) {
        if (!isPlainName(std::string_view(address).substr(kInProcessPrefix.size())))
            return makeError(ErrorCode::InvalidArgument, "invalid in-process endpoint: " + address);
        return ConnectionEndpoint(std::move(address), EndpointKind::InProcess);
    }
    // sockaddr_un::sun_path is 104 bytes on macOS (108 on Linux), NUL included.
    if (!address.empty() && address.front() == '/' && address.size() < 104 && address.find('\0') == std::string::npos)
        return ConnectionEndpoint(std::move(address), EndpointKind::UnixSocket);
    return makeError(ErrorCode::InvalidArgument, "unrecognised IPC endpoint: '" + address + "'");
}

Result<ConnectionEndpoint> ConnectionEndpoint::fromEnvironment(const char* variable) {
    std::optional<std::string> value = readEnvironment(variable);
    if (!value) return makeError(ErrorCode::NotFound, std::string("environment variable ") + variable + " is not set");
    return parse(std::move(*value));
}

std::optional<std::string> readEnvironment(const char* name) {
#if defined(_WIN32)
    char* buffer = nullptr;
    std::size_t length = 0;
    if (_dupenv_s(&buffer, &length, name) != 0 || buffer == nullptr) return std::nullopt;
    std::string value(buffer);
    std::free(buffer);
#else
    const char* raw = std::getenv(name);
    if (raw == nullptr) return std::nullopt;
    std::string value(raw);
#endif
    if (value.empty()) return std::nullopt;
    return value;
}

std::chrono::milliseconds readEnvironmentMillis(const char* name, std::chrono::milliseconds fallback) {
    const std::optional<std::string> value = readEnvironment(name);
    if (!value) return fallback;
    char* end = nullptr;
    const long long ms = std::strtoll(value->c_str(), &end, 10);
    if (end == value->c_str() || *end != '\0' || ms < 0 || ms > 24LL * 3600 * 1000) return fallback;
    return std::chrono::milliseconds{ms};
}

}  // namespace wpiemu::ipc
