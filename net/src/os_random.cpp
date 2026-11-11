#include "wpiemu/net/os_random.h"

#include <algorithm>
#include <cerrno>
#include <string>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <bcrypt.h>
#else
#include <sys/random.h>
#include <unistd.h>
#endif

namespace wpiemu::net {

namespace {

sdk::Error randomError(const std::string& what) {
    return sdk::makeError(sdk::ErrorCode::IoError, "operating system random source failed: " + what);
}

}  // namespace

#if defined(_WIN32)

sdk::Status fillRandom(std::span<std::byte> out) {
    std::size_t done = 0;
    while (done < out.size()) {
        const auto chunk = static_cast<ULONG>(std::min<std::size_t>(out.size() - done, 1u << 20));
        const NTSTATUS status = BCryptGenRandom(nullptr, reinterpret_cast<PUCHAR>(out.data() + done), chunk,
                                                BCRYPT_USE_SYSTEM_PREFERRED_RNG);
        if (status != 0) return randomError("BCryptGenRandom returned " + std::to_string(status));
        done += chunk;
    }
    return sdk::okStatus();
}

#else

sdk::Status fillRandom(std::span<std::byte> out) {
    constexpr std::size_t kGetentropyMax = 256;
    std::size_t done = 0;
    bool useGetrandom = false;
    while (done < out.size()) {
        const std::size_t chunk = std::min(out.size() - done, kGetentropyMax);
        void* const dst = out.data() + done;
        if (!useGetrandom) {
            if (getentropy(dst, chunk) == 0) {
                done += chunk;
                continue;
            }
#if defined(__linux__)
            if (errno == ENOSYS) {
                useGetrandom = true;
                continue;
            }
#endif
            if (errno == EINTR) continue;
            return randomError("getentropy failed, errno " + std::to_string(errno));
        }
#if defined(__linux__)
        const ssize_t got = getrandom(dst, chunk, 0);
        if (got < 0) {
            if (errno == EINTR) continue;
            return randomError("getrandom failed, errno " + std::to_string(errno));
        }
        done += static_cast<std::size_t>(got);
#else
        return randomError("no entropy source");
#endif
    }
    return sdk::okStatus();
}

#endif

}  // namespace wpiemu::net
