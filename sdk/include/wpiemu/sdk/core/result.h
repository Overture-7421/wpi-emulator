#pragma once

// Error-handling contract for the whole SDK.
//
//  * Recoverable / expected failures (bad config, missing device, transport closed, plugin
//    refuses a request) are reported by value as `Result<T>` / `Status`. SDK interfaces and
//    implementations of them MUST NOT let exceptions escape across an interface boundary
//    (treat every SDK virtual as `noexcept`-in-spirit; catch at the boundary of third-party code,
//    e.g. around JSON parsing or Qt calls, and convert to `Error`).
//  * Programmer errors (contract violations: dereferencing an errored Result, invalid ids passed
//    to a physics call in debug builds) terminate via `contractViolation`; they are not recoverable.
//  * Hot-path per-tick calls (physics queries, hub reads) do not return Result; they follow the
//    documented "invalid id => neutral value" rule of the individual interface.

#include <cstdlib>
#include <string>
#include <utility>
#include <variant>

namespace wpiemu::sdk {

enum class ErrorCode {
    InvalidArgument,
    NotFound,
    AlreadyExists,
    NotImplemented,
    Unsupported,
    InvalidState,
    OutOfRange,
    IoError,
    ParseError,
    ProtocolError,
    VersionMismatch,
    Timeout,
    Disconnected,
    Cancelled,
    Internal,
};

struct Error {
    ErrorCode code = ErrorCode::Internal;
    std::string message;

    Error() = default;
    Error(ErrorCode c, std::string msg) : code(c), message(std::move(msg)) {}
};

[[noreturn]] inline void contractViolation(const char* what) noexcept {
    // Deliberately no exception: contract violations are bugs, not runtime conditions.
    (void)what;
    std::abort();
}

// std::expected-like value-or-error (C++20 has no std::expected).
template <class T>
class [[nodiscard]] Result {
public:
    Result(T value) : v_(std::in_place_index<0>, std::move(value)) {}      // NOLINT: implicit by design
    Result(Error error) : v_(std::in_place_index<1>, std::move(error)) {}  // NOLINT

    bool ok() const noexcept { return v_.index() == 0; }
    explicit operator bool() const noexcept { return ok(); }

    T& value() & {
        if (!ok()) contractViolation("Result::value() on error");
        return std::get<0>(v_);
    }
    const T& value() const& {
        if (!ok()) contractViolation("Result::value() on error");
        return std::get<0>(v_);
    }
    T&& value() && {
        if (!ok()) contractViolation("Result::value() on error");
        return std::get<0>(std::move(v_));
    }
    const Error& error() const {
        if (ok()) contractViolation("Result::error() on success");
        return std::get<1>(v_);
    }
    T valueOr(T fallback) const& { return ok() ? std::get<0>(v_) : std::move(fallback); }

private:
    std::variant<T, Error> v_;
};

template <>
class [[nodiscard]] Result<void> {
public:
    Result() = default;
    Result(Error error) : error_(std::move(error)), ok_(false) {}  // NOLINT

    bool ok() const noexcept { return ok_; }
    explicit operator bool() const noexcept { return ok_; }
    const Error& error() const {
        if (ok_) contractViolation("Status::error() on success");
        return error_;
    }

private:
    Error error_;
    bool ok_ = true;
};

using Status = Result<void>;

inline Status okStatus() { return Status{}; }
inline Error makeError(ErrorCode code, std::string message) { return Error{code, std::move(message)}; }

}  // namespace wpiemu::sdk
