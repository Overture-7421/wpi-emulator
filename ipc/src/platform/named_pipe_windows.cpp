// Windows transport: overlapped named pipes (byte mode; framing is FramedTransport's job).
//
// Why a named pipe and not TCP loopback (docs/reference/extension-ipc.md): it is never reachable from the network (the
// server rejects remote clients), the name is unguessable and the first instance is created with
// FILE_FLAG_FIRST_PIPE_INSTANCE (no squatting), the DACL admits only the current user, there are no
// ports to collide or firewall prompts, and a ping-pong costs ~20-40 us.
#include <algorithm>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <sddl.h>

#include "platform_common.h"
#include "platform_ipc.h"
#include "wpiemu/ipc/byte_stream.h"

namespace wpiemu::ipc::platform {

using namespace sdk;
using namespace sdk::proto;

namespace {

constexpr DWORD kPipeBufferBytes = 64 * 1024;
constexpr DWORD kMaxWriteChunk = 1u << 20;

std::wstring widen(const std::string& ascii) { return std::wstring(ascii.begin(), ascii.end()); }

Error windowsError(ErrorCode code, const std::string& what, DWORD error) {
    return makeError(code, what + " (Win32 error " + std::to_string(error) + ")");
}

bool isPeerGone(DWORD e) {
    return e == ERROR_BROKEN_PIPE || e == ERROR_NO_DATA || e == ERROR_PIPE_NOT_CONNECTED || e == ERROR_NETNAME_DELETED ||
           e == ERROR_OPERATION_ABORTED || e == ERROR_INVALID_HANDLE;
}

Error ioError(const char* op, DWORD e) {
    return isPeerGone(e) ? makeError(ErrorCode::Disconnected, std::string("pipe closed during ") + op)
                         : windowsError(ErrorCode::IoError, std::string("pipe ") + op + " failed", e);
}

DWORD waitMs(std::chrono::steady_clock::time_point deadline) { return static_cast<DWORD>(boundedWaitMs(deadline)); }

HANDLE makeEvent() { return CreateEventW(nullptr, TRUE, FALSE, nullptr); }

// DACL granting full control to the current user only (best effort: null = default security).
class OwnerOnlySecurity {
public:
    OwnerOnlySecurity() {
        HANDLE token = nullptr;
        if (!OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &token)) return;
        DWORD length = 0;
        GetTokenInformation(token, TokenUser, nullptr, 0, &length);
        std::vector<BYTE> buffer(length);
        if (length != 0 && GetTokenInformation(token, TokenUser, buffer.data(), length, &length)) {
            LPWSTR sid = nullptr;
            if (ConvertSidToStringSidW(reinterpret_cast<TOKEN_USER*>(buffer.data())->User.Sid, &sid)) {
                const std::wstring sddl = L"D:P(A;;GA;;;" + std::wstring(sid) + L")";
                LocalFree(sid);
                if (ConvertStringSecurityDescriptorToSecurityDescriptorW(sddl.c_str(), SDDL_REVISION_1, &descriptor_, nullptr)) {
                    attributes_.nLength = sizeof(attributes_);
                    attributes_.lpSecurityDescriptor = descriptor_;
                    attributes_.bInheritHandle = FALSE;
                    valid_ = true;
                }
            }
        }
        CloseHandle(token);
    }
    ~OwnerOnlySecurity() {
        if (descriptor_ != nullptr) LocalFree(descriptor_);
    }
    OwnerOnlySecurity(const OwnerOnlySecurity&) = delete;
    OwnerOnlySecurity& operator=(const OwnerOnlySecurity&) = delete;

    SECURITY_ATTRIBUTES* get() { return valid_ ? &attributes_ : nullptr; }

private:
    SECURITY_ATTRIBUTES attributes_{};
    PSECURITY_DESCRIPTOR descriptor_ = nullptr;
    bool valid_ = false;
};

class PipeStream final : public IByteStream {
public:
    // Takes ownership of `pipe` (opened with FILE_FLAG_OVERLAPPED). Null on failure (pipe closed).
    static std::unique_ptr<PipeStream> adopt(HANDLE pipe, std::string label) {
        std::unique_ptr<PipeStream> s(new PipeStream(pipe, std::move(label)));
        if (s->wake_ == nullptr || s->readEvent_ == nullptr || s->writeEvent_ == nullptr) return nullptr;
        return s;
    }

    ~PipeStream() override {
        close();
        for (HANDLE h : {wake_, readEvent_, writeEvent_})
            if (h != nullptr) CloseHandle(h);
    }

    Result<std::size_t> read(std::span<std::byte> buffer, std::chrono::milliseconds timeout) override {
        Inflight inflight(inflight_);
        if (closing_) return makeError(ErrorCode::Disconnected, "pipe closed");
        if (buffer.empty()) return std::size_t{0};
        if (carryPos_ < carry_.size()) return deliverCarry(buffer);

        const auto deadline = std::chrono::steady_clock::now() + timeout;
        for (;;) {
            if (!readPending_) {
                ResetEvent(readEvent_);
                readOverlapped_ = OVERLAPPED{};
                readOverlapped_.hEvent = readEvent_;
                DWORD got = 0;
                if (ReadFile(handle_, readBuffer_.data(), static_cast<DWORD>(readBuffer_.size()), &got, &readOverlapped_)) {
                    if (got == 0) continue;
                    return deliver(buffer, got);
                }
                const DWORD e = GetLastError();
                if (e != ERROR_IO_PENDING) return ioError("read", e);
                readPending_ = true;
            }
            const HANDLE handles[2] = {readEvent_, wake_};
            const DWORD w = WaitForMultipleObjects(2, handles, FALSE, waitMs(deadline));
            if (w == WAIT_TIMEOUT) return std::size_t{0};  // the read stays pending for the next call
            if (w == WAIT_OBJECT_0 + 1) return makeError(ErrorCode::Disconnected, "pipe closed");
            if (w != WAIT_OBJECT_0) return windowsError(ErrorCode::IoError, "pipe wait failed", GetLastError());
            DWORD got = 0;
            const BOOL ok = GetOverlappedResult(handle_, &readOverlapped_, &got, FALSE);
            readPending_ = false;
            if (!ok) return ioError("read", GetLastError());
            if (got != 0) return deliver(buffer, got);
        }
    }

    Status write(std::span<const std::byte> data, std::chrono::milliseconds timeout) override {
        Inflight inflight(inflight_);
        if (closing_) return makeError(ErrorCode::Disconnected, "pipe closed");
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        std::size_t offset = 0;
        while (offset < data.size()) {
            ResetEvent(writeEvent_);
            OVERLAPPED ov{};
            ov.hEvent = writeEvent_;
            const DWORD chunk = static_cast<DWORD>(std::min<std::size_t>(data.size() - offset, kMaxWriteChunk));
            DWORD wrote = 0;
            if (!WriteFile(handle_, data.data() + offset, chunk, &wrote, &ov)) {
                const DWORD e = GetLastError();
                if (e != ERROR_IO_PENDING) return ioError("write", e);
                const HANDLE handles[2] = {writeEvent_, wake_};
                const DWORD w = WaitForMultipleObjects(2, handles, FALSE, waitMs(deadline));
                if (w != WAIT_OBJECT_0) {
                    CancelIoEx(handle_, &ov);
                    DWORD ignored = 0;
                    GetOverlappedResult(handle_, &ov, &ignored, TRUE);  // `ov` must not leave scope while pending
                    return w == WAIT_TIMEOUT ? makeError(ErrorCode::Timeout, "peer is not reading the pipe")
                                             : makeError(ErrorCode::Disconnected, "pipe closed");
                }
                if (!GetOverlappedResult(handle_, &ov, &wrote, FALSE)) return ioError("write", GetLastError());
            }
            offset += wrote;
        }
        return okStatus();
    }

    void close() override {
        std::lock_guard lock(closeMutex_);
        closing_ = true;
        SetEvent(wake_);
        Inflight::waitUntilIdle(inflight_);
        if (handle_ == INVALID_HANDLE_VALUE) return;
        if (readPending_) {
            CancelIoEx(handle_, &readOverlapped_);
            DWORD ignored = 0;
            GetOverlappedResult(handle_, &readOverlapped_, &ignored, TRUE);
            readPending_ = false;
        }
        CloseHandle(handle_);  // the peer sees EOF once it has drained what was already written
        handle_ = INVALID_HANDLE_VALUE;
    }

    bool isOpen() const override { return !closing_; }
    std::string describe() const override { return label_; }

private:
    PipeStream(HANDLE pipe, std::string label)
        : handle_(pipe), label_(std::move(label)), readBuffer_(kPipeBufferBytes), wake_(makeEvent()),
          readEvent_(makeEvent()), writeEvent_(makeEvent()) {}

    Result<std::size_t> deliver(std::span<std::byte> buffer, DWORD got) {
        const std::size_t n = std::min<std::size_t>(buffer.size(), got);
        std::copy_n(readBuffer_.begin(), n, buffer.begin());
        if (n < got) {
            carry_.assign(readBuffer_.begin() + static_cast<std::ptrdiff_t>(n), readBuffer_.begin() + static_cast<std::ptrdiff_t>(got));
            carryPos_ = 0;
        }
        return n;
    }

    Result<std::size_t> deliverCarry(std::span<std::byte> buffer) {
        const std::size_t n = std::min(buffer.size(), carry_.size() - carryPos_);
        std::copy_n(carry_.begin() + static_cast<std::ptrdiff_t>(carryPos_), n, buffer.begin());
        carryPos_ += n;
        return n;
    }

    HANDLE handle_;
    std::string label_;
    std::vector<std::byte> readBuffer_;
    std::vector<std::byte> carry_;  // bytes of a completed read that did not fit the caller's buffer
    std::size_t carryPos_ = 0;
    HANDLE wake_;
    HANDLE readEvent_;
    HANDLE writeEvent_;
    OVERLAPPED readOverlapped_{};
    bool readPending_ = false;
    std::atomic<bool> closing_{false};
    std::atomic<int> inflight_{0};
    std::mutex closeMutex_;
};

std::unique_ptr<ITransport> makeTransport(HANDLE pipe, const std::string& label, const FramedTransportOptions& options) {
    std::unique_ptr<PipeStream> stream = PipeStream::adopt(pipe, label);
    if (!stream) return nullptr;  // adopt() already closed the pipe
    return std::make_unique<FramedTransport>(std::move(stream), options);
}

class NamedPipeServer final : public ITransportServer {
public:
    NamedPipeServer(std::string address, FramedTransportOptions options)
        : address_(std::move(address)), options_(options), connectEvent_(makeEvent()), wake_(makeEvent()) {}

    ~NamedPipeServer() override {
        close();
        for (HANDLE h : {connectEvent_, wake_})
            if (h != nullptr) CloseHandle(h);
    }

    Status listen() override {
        if (connectEvent_ == nullptr || wake_ == nullptr) return makeError(ErrorCode::Internal, "cannot create events");
        std::lock_guard lock(acceptMutex_);
        if (pending_ != INVALID_HANDLE_VALUE) return makeError(ErrorCode::InvalidState, "already listening");
        std::string reason;
        ErrorCode code = ErrorCode::IoError;
        pending_ = createInstance(true, reason, code);
        if (pending_ == INVALID_HANDLE_VALUE) return makeError(code, reason);
        return okStatus();
    }

    std::string endpoint() const override { return address_; }

    Result<std::unique_ptr<ITransport>> accept(std::chrono::milliseconds timeout) override {
        std::lock_guard lock(acceptMutex_);
        if (closing_) return makeError(ErrorCode::Cancelled, "server closed");
        if (pending_ == INVALID_HANDLE_VALUE) return makeError(ErrorCode::InvalidState, "not listening");

        ResetEvent(connectEvent_);
        OVERLAPPED ov{};
        ov.hEvent = connectEvent_;
        bool connected = false;
        if (ConnectNamedPipe(pending_, &ov)) {
            connected = true;
        } else {
            const DWORD e = GetLastError();
            if (e == ERROR_PIPE_CONNECTED) {
                connected = true;  // a client connected before we asked
            } else if (e != ERROR_IO_PENDING) {
                return windowsError(ErrorCode::IoError, "ConnectNamedPipe failed", e);
            } else {
                const HANDLE handles[2] = {connectEvent_, wake_};
                const DWORD w = WaitForMultipleObjects(2, handles, FALSE, waitMs(std::chrono::steady_clock::now() + timeout));
                DWORD ignored = 0;
                if (w == WAIT_OBJECT_0) {
                    connected = GetOverlappedResult(pending_, &ov, &ignored, FALSE) != FALSE;
                } else {
                    CancelIoEx(pending_, &ov);
                    connected = GetOverlappedResult(pending_, &ov, &ignored, TRUE) != FALSE;  // a client may have won the race
                    if (!connected && closing_) return makeError(ErrorCode::Cancelled, "server closed");
                }
                if (!connected) return std::unique_ptr<ITransport>{};  // timeout
            }
        }

        HANDLE accepted = pending_;
        std::string ignoredReason;
        ErrorCode ignoredCode = ErrorCode::IoError;
        pending_ = closing_ ? INVALID_HANDLE_VALUE : createInstance(false, ignoredReason, ignoredCode);  // keep the name reserved
        std::unique_ptr<ITransport> transport = makeTransport(accepted, address_, options_);
        if (!transport) {
            return makeError(ErrorCode::Internal, "cannot create pipe stream");
        }
        return transport;
    }

    void close() override {
        closing_ = true;
        if (wake_ != nullptr) SetEvent(wake_);
        std::lock_guard lock(acceptMutex_);
        if (pending_ != INVALID_HANDLE_VALUE) {
            CloseHandle(pending_);
            pending_ = INVALID_HANDLE_VALUE;
        }
    }

private:
    HANDLE createInstance(bool first, std::string& reason, ErrorCode& code) {
        const DWORD openMode = PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED | (first ? FILE_FLAG_FIRST_PIPE_INSTANCE : 0);
        const DWORD pipeMode = PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT | PIPE_REJECT_REMOTE_CLIENTS;
        const HANDLE h = CreateNamedPipeW(widen(address_).c_str(), openMode, pipeMode, PIPE_UNLIMITED_INSTANCES,
                                          kPipeBufferBytes, kPipeBufferBytes, 0, security_.get());
        if (h == INVALID_HANDLE_VALUE) {
            const DWORD e = GetLastError();
            const bool inUse = first && (e == ERROR_ACCESS_DENIED || e == ERROR_PIPE_BUSY);
            code = inUse ? ErrorCode::AlreadyExists : ErrorCode::IoError;
            reason = (inUse ? "endpoint already in use: " : "cannot create pipe: ") + address_ + " (Win32 error " +
                     std::to_string(e) + ")";
        }
        return h;
    }

    std::string address_;
    FramedTransportOptions options_;
    OwnerOnlySecurity security_;
    HANDLE connectEvent_;
    HANDLE wake_;
    HANDLE pending_ = INVALID_HANDLE_VALUE;  // a pipe instance already waiting for the next client
    std::mutex acceptMutex_;
    std::atomic<bool> closing_{false};
};

class NamedPipeClient final : public ITransportClient {
public:
    explicit NamedPipeClient(FramedTransportOptions options) : options_(options) {}

    Result<std::unique_ptr<ITransport>> connect(const std::string& endpoint, std::chrono::milliseconds timeout) override {
        const sdk::Result<ConnectionEndpoint> parsed = parseEndpointOfKind(endpoint, EndpointKind::NamedPipe, "named-pipe");
        if (!parsed) return parsed.error();

        const std::wstring name = widen(endpoint);
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        for (;;) {
            // SQOS_IDENTIFICATION: the server may learn who we are but cannot impersonate us.
            const HANDLE h = CreateFileW(name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
                                         FILE_FLAG_OVERLAPPED | SECURITY_SQOS_PRESENT | SECURITY_IDENTIFICATION, nullptr);
            if (h != INVALID_HANDLE_VALUE) {
                std::unique_ptr<ITransport> transport = makeTransport(h, endpoint, options_);
                if (!transport) {
                    return makeError(ErrorCode::Internal, "cannot create pipe stream");
                }
                return transport;
            }
            const DWORD e = GetLastError();
            if (e == ERROR_PIPE_BUSY) {
                WaitNamedPipeW(name.c_str(), std::min<DWORD>(waitMs(deadline), 50));
            } else if (e == ERROR_FILE_NOT_FOUND) {
                std::this_thread::sleep_for(kConnectRetryInterval);  // the server is not listening yet
            } else {
                return windowsError(ErrorCode::IoError, "cannot open " + endpoint, e);
            }
            if (std::chrono::steady_clock::now() >= deadline) return connectTimedOut(endpoint);
        }
    }

private:
    FramedTransportOptions options_;
};

}  // namespace

EndpointKind nativeKind() { return EndpointKind::NamedPipe; }

std::string nativeAddressFor(const std::string& token) { return std::string(kNamedPipePrefix) + "wpiemu-" + token; }

std::unique_ptr<ITransportServer> makeNativeServer(std::string address, FramedTransportOptions options) {
    return std::make_unique<NamedPipeServer>(std::move(address), options);
}

std::unique_ptr<ITransportClient> makeNativeClient(FramedTransportOptions options) {
    return std::make_unique<NamedPipeClient>(options);
}

}  // namespace wpiemu::ipc::platform
