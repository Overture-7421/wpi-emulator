// POSIX transport (Linux, macOS): unix domain stream sockets in a private (0700) directory.
// Framing is FramedTransport's job; this file only moves bytes with poll()-based timeouts and a
// self-pipe so that close() from any thread wakes blocked readers/writers/acceptors.
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include "platform_ipc.h"
#include "wpiemu/ipc/byte_stream.h"

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0  // macOS: SO_NOSIGPIPE is set on the socket instead
#endif

namespace wpiemu::ipc::platform {

using namespace sdk;
using namespace sdk::proto;

namespace {

Error errnoError(ErrorCode code, const std::string& what, int e) { return makeError(code, what + ": " + std::strerror(e)); }

bool isPeerGone(int e) { return e == EPIPE || e == ECONNRESET || e == ENOTCONN || e == EBADF || e == ECONNABORTED; }

int pollMs(std::chrono::steady_clock::time_point deadline) {
    return static_cast<int>(std::min<long long>(proto::timeLeft(deadline).count(), 0x7FFFFFFF));
}

bool setNonBlockingCloexec(int fd) {
    const int fl = fcntl(fd, F_GETFL, 0);
    const int fd_flags = fcntl(fd, F_GETFD, 0);
    return fl >= 0 && fd_flags >= 0 && fcntl(fd, F_SETFL, fl | O_NONBLOCK) == 0 && fcntl(fd, F_SETFD, fd_flags | FD_CLOEXEC) == 0;
}

// Self-pipe: becomes (and stays) readable once wake() was called.
class WakePipe {
public:
    WakePipe() {
        if (::pipe(fds_) != 0) {
            fds_[0] = fds_[1] = -1;
            return;
        }
        for (int fd : fds_) setNonBlockingCloexec(fd);
    }
    ~WakePipe() {
        for (int fd : fds_)
            if (fd >= 0) ::close(fd);
    }
    WakePipe(const WakePipe&) = delete;
    WakePipe& operator=(const WakePipe&) = delete;

    bool valid() const { return fds_[0] >= 0; }
    int readFd() const { return fds_[0]; }
    void wake() {
        const char byte = 1;
        [[maybe_unused]] const ssize_t n = ::write(fds_[1], &byte, 1);
    }

private:
    int fds_[2] = {-1, -1};
};

sockaddr_un makeAddress(const std::string& path) {
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::memcpy(addr.sun_path, path.c_str(), path.size() + 1);  // length validated by ConnectionEndpoint::parse
    return addr;
}

class SocketStream final : public IByteStream {
public:
    SocketStream(int fd, std::string label) : fd_(fd), label_(std::move(label)) {}
    ~SocketStream() override { close(); }

    bool usable() const { return wake_.valid(); }

    Result<std::size_t> read(std::span<std::byte> buffer, std::chrono::milliseconds timeout) override {
        Inflight inflight(inflight_);
        if (closing_) return makeError(ErrorCode::Disconnected, "socket closed");
        if (buffer.empty()) return std::size_t{0};
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        for (;;) {
            pollfd fds[2] = {{fd_, POLLIN, 0}, {wake_.readFd(), POLLIN, 0}};
            const int r = ::poll(fds, 2, pollMs(deadline));
            if (r < 0) {
                if (errno == EINTR) continue;
                return errnoError(ErrorCode::IoError, "poll failed", errno);
            }
            if (r == 0) return std::size_t{0};
            if ((fds[1].revents & POLLIN) != 0) return makeError(ErrorCode::Disconnected, "socket closed");
            const ssize_t n = ::recv(fd_, buffer.data(), buffer.size(), 0);
            if (n > 0) return static_cast<std::size_t>(n);
            if (n == 0) return makeError(ErrorCode::Disconnected, "peer closed the connection");
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            if (isPeerGone(errno)) return makeError(ErrorCode::Disconnected, "peer closed the connection");
            return errnoError(ErrorCode::IoError, "recv failed", errno);
        }
    }

    Status write(std::span<const std::byte> data, std::chrono::milliseconds timeout) override {
        Inflight inflight(inflight_);
        if (closing_) return makeError(ErrorCode::Disconnected, "socket closed");
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        std::size_t offset = 0;
        while (offset < data.size()) {
            const ssize_t n = ::send(fd_, data.data() + offset, data.size() - offset, MSG_NOSIGNAL);
            if (n > 0) {
                offset += static_cast<std::size_t>(n);
                continue;
            }
            if (n < 0 && isPeerGone(errno)) return makeError(ErrorCode::Disconnected, "peer closed the connection");
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
                return errnoError(ErrorCode::IoError, "send failed", errno);
            pollfd fds[2] = {{fd_, POLLOUT, 0}, {wake_.readFd(), POLLIN, 0}};
            const int r = ::poll(fds, 2, pollMs(deadline));
            if (r < 0 && errno != EINTR) return errnoError(ErrorCode::IoError, "poll failed", errno);
            if (r == 0) return makeError(ErrorCode::Timeout, "peer is not reading the socket");
            if (r > 0 && (fds[1].revents & POLLIN) != 0) return makeError(ErrorCode::Disconnected, "socket closed");
        }
        return okStatus();
    }

    void close() override {
        std::lock_guard lock(closeMutex_);
        closing_ = true;
        wake_.wake();
        if (fd_ >= 0) ::shutdown(fd_, SHUT_RDWR);  // the peer sees EOF immediately
        while (inflight_.load() != 0) std::this_thread::sleep_for(std::chrono::microseconds(50));
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool isOpen() const override { return !closing_; }
    std::string describe() const override { return label_; }

private:
    struct Inflight {
        explicit Inflight(std::atomic<int>& n) : n_(n) { ++n_; }
        ~Inflight() { --n_; }
        Inflight(const Inflight&) = delete;
        Inflight& operator=(const Inflight&) = delete;
        std::atomic<int>& n_;
    };

    int fd_;
    std::string label_;
    WakePipe wake_;
    std::atomic<bool> closing_{false};
    std::atomic<int> inflight_{0};
    std::mutex closeMutex_;
};

std::unique_ptr<ITransport> makeTransport(int fd, const std::string& label, const FramedTransportOptions& options) {
    if (!setNonBlockingCloexec(fd)) {
        ::close(fd);
        return nullptr;
    }
#ifdef SO_NOSIGPIPE
    const int on = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &on, sizeof(on));
#endif
    auto stream = std::make_unique<SocketStream>(fd, label);  // owns fd from here on
    if (!stream->usable()) return nullptr;
    return std::make_unique<FramedTransport>(std::move(stream), options);
}

// True when a live server accepts connections on the unix socket at `path` (as opposed to a stale file).
bool isListening(const std::string& path) {
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return false;
    setNonBlockingCloexec(fd);
    const sockaddr_un addr = makeAddress(path);
    const bool alive = ::connect(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) == 0 || errno == EAGAIN;
    ::close(fd);
    return alive;
}

std::string parentDirectory(const std::string& path) {
    const std::size_t slash = path.find_last_of('/');
    return slash == std::string::npos || slash == 0 ? std::string("/") : path.substr(0, slash);
}

class UnixSocketServer final : public ITransportServer {
public:
    UnixSocketServer(std::string address, FramedTransportOptions options) : address_(std::move(address)), options_(options) {}
    ~UnixSocketServer() override { close(); }

    Status listen() override {
        std::lock_guard lock(acceptMutex_);
        if (listenFd_ >= 0) return makeError(ErrorCode::InvalidState, "already listening");
        if (!wake_.valid()) return makeError(ErrorCode::Internal, "cannot create wake pipe");

        // The socket lives in a directory only we can enter: other users cannot connect or squat the name.
        const std::string dir = parentDirectory(address_);
        if (::mkdir(dir.c_str(), 0700) == 0) {
            createdDirectory_ = dir;
        } else if (errno == EEXIST) {
            struct stat st {};
            if (::lstat(dir.c_str(), &st) != 0 || !S_ISDIR(st.st_mode) || st.st_uid != ::geteuid() || (st.st_mode & 077) != 0)
                return makeError(ErrorCode::AlreadyExists, "unsafe existing directory: " + dir);
        } else {
            return errnoError(ErrorCode::IoError, "cannot create " + dir, errno);
        }

        const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd < 0) return fail(errnoError(ErrorCode::IoError, "socket failed", errno));
        if (!setNonBlockingCloexec(fd)) {
            ::close(fd);
            return fail(errnoError(ErrorCode::IoError, "fcntl failed", errno));
        }
        struct stat existing {};
        if (::lstat(address_.c_str(), &existing) == 0 && S_ISSOCK(existing.st_mode) && existing.st_uid == ::geteuid()) {
            if (isListening(address_)) {
                ::close(fd);
                return fail(makeError(ErrorCode::AlreadyExists, "endpoint already in use: " + address_));
            }
            ::unlink(address_.c_str());  // stale socket of a crashed emulator
        }
        const sockaddr_un addr = makeAddress(address_);
        if (::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
            const int e = errno;
            ::close(fd);
            return fail(errnoError(e == EADDRINUSE ? ErrorCode::AlreadyExists : ErrorCode::IoError, "cannot bind " + address_, e));
        }
        ::chmod(address_.c_str(), 0600);
        if (::listen(fd, 4) != 0) {
            const int e = errno;
            ::close(fd);
            ::unlink(address_.c_str());
            return fail(errnoError(ErrorCode::IoError, "listen failed", e));
        }
        listenFd_ = fd;
        return okStatus();
    }

    std::string endpoint() const override { return address_; }

    Result<std::unique_ptr<ITransport>> accept(std::chrono::milliseconds timeout) override {
        std::lock_guard lock(acceptMutex_);
        if (closing_) return makeError(ErrorCode::Cancelled, "server closed");
        if (listenFd_ < 0) return makeError(ErrorCode::InvalidState, "not listening");
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        for (;;) {
            pollfd fds[2] = {{listenFd_, POLLIN, 0}, {wake_.readFd(), POLLIN, 0}};
            const int r = ::poll(fds, 2, pollMs(deadline));
            if (r < 0) {
                if (errno == EINTR) continue;
                return errnoError(ErrorCode::IoError, "poll failed", errno);
            }
            if (r == 0) return std::unique_ptr<ITransport>{};
            if ((fds[1].revents & POLLIN) != 0) return makeError(ErrorCode::Cancelled, "server closed");
            const int fd = ::accept(listenFd_, nullptr, nullptr);
            if (fd < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR || errno == ECONNABORTED) continue;
                return errnoError(ErrorCode::IoError, "accept failed", errno);
            }
            std::unique_ptr<ITransport> transport = makeTransport(fd, address_, options_);
            if (!transport) return makeError(ErrorCode::Internal, "cannot create socket stream");
            return transport;
        }
    }

    void close() override {
        closing_ = true;
        wake_.wake();
        std::lock_guard lock(acceptMutex_);
        if (listenFd_ >= 0) {
            ::close(listenFd_);
            listenFd_ = -1;
            ::unlink(address_.c_str());
        }
        if (!createdDirectory_.empty()) {
            ::rmdir(createdDirectory_.c_str());
            createdDirectory_.clear();
        }
    }

private:
    Status fail(Error error) {
        if (!createdDirectory_.empty()) {
            ::rmdir(createdDirectory_.c_str());
            createdDirectory_.clear();
        }
        return error;
    }

    std::string address_;
    FramedTransportOptions options_;
    WakePipe wake_;
    int listenFd_ = -1;
    std::string createdDirectory_;
    std::mutex acceptMutex_;
    std::atomic<bool> closing_{false};
};

class UnixSocketClient final : public ITransportClient {
public:
    explicit UnixSocketClient(FramedTransportOptions options) : options_(options) {}

    Result<std::unique_ptr<ITransport>> connect(const std::string& endpoint, std::chrono::milliseconds timeout) override {
        Result<ConnectionEndpoint> parsed = ConnectionEndpoint::parse(endpoint);
        if (!parsed) return parsed.error();
        if (parsed.value().kind() != EndpointKind::UnixSocket)
            return makeError(ErrorCode::InvalidArgument, "not a unix-socket endpoint: " + endpoint);

        const sockaddr_un addr = makeAddress(endpoint);
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        for (;;) {
            const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
            if (fd < 0) return errnoError(ErrorCode::IoError, "socket failed", errno);
            if (!setNonBlockingCloexec(fd)) {
                const int e = errno;
                ::close(fd);
                return errnoError(ErrorCode::IoError, "fcntl failed", e);
            }
            if (::connect(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) == 0) {
                std::unique_ptr<ITransport> transport = makeTransport(fd, endpoint, options_);
                if (!transport) return makeError(ErrorCode::Internal, "cannot create socket stream");
                return transport;
            }
            const int e = errno;
            ::close(fd);
            // Not listening yet / backlog full: keep trying until the deadline.
            if (e != ENOENT && e != ECONNREFUSED && e != EAGAIN && e != EINTR)
                return errnoError(ErrorCode::IoError, "cannot connect to " + endpoint, e);
            if (std::chrono::steady_clock::now() >= deadline)
                return makeError(ErrorCode::Timeout, "timed out connecting to " + endpoint);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

private:
    FramedTransportOptions options_;
};

}  // namespace

EndpointKind nativeKind() { return EndpointKind::UnixSocket; }

std::string nativeAddressFor(const std::string& token) {
    // Short, private base directory: sun_path is only 104 bytes on macOS.
    std::string base = "/tmp";
    for (const char* var : {"XDG_RUNTIME_DIR", "TMPDIR"}) {
        const std::optional<std::string> value = sdk::readEnvironment(var);
        if (value && value->front() == '/') {
            std::string candidate = *value;
            while (candidate.size() > 1 && candidate.back() == '/') candidate.pop_back();
            if (candidate.size() + 1 + 7 + token.size() + 9 < 100) {
                base = std::move(candidate);
                break;
            }
        }
    }
    return base + "/wpiemu-" + token + "/ipc.sock";
}

std::unique_ptr<ITransportServer> makeNativeServer(std::string address, FramedTransportOptions options) {
    return std::make_unique<UnixSocketServer>(std::move(address), options);
}

std::unique_ptr<ITransportClient> makeNativeClient(FramedTransportOptions options) {
    return std::make_unique<UnixSocketClient>(options);
}

}  // namespace wpiemu::ipc::platform
