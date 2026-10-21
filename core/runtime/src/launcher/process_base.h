#pragma once

// The part of a supervised process that is the same on every OS (internal to core/runtime/src/launcher): the state shared
// by the caller and the supervision threads, the queries, the two output readers and the exit monitor. process_posix.cpp and
// process_win.cpp keep only what the OS dictates: spawning, signalling/killing, and reading and waiting on native handles.

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "wpiemu/core/launcher/process.h"

namespace wpiemu::core::launcher {

class ProcessBase : public IRobotProcess {
public:
    bool running() const override;
    std::int64_t pid() const override;
    std::optional<ExitInfo> exitInfo() const override;
    bool waitForExit(std::chrono::milliseconds timeout) override;

protected:
    // Derived destructors call kill() and then joinAll() (the base cannot call the derived kill()).
    ~ProcessBase() override = default;

    // First step of start(), with mutex_ held: refuses a second start and a start after kill(), and records the sink.
    sdk::Status beginStart(ILogSink& sink);
    // Last step of a successful start(), with mutex_ held and the native handles already stored: publishes the pid and
    // starts the two reader threads and the exit monitor.
    void startSupervision(std::int64_t pid);
    void joinAll();

    // ---- what the OS provides ----
    // At most `capacity` bytes of the stream. > 0: bytes read; 0: end of stream; < 0: failure. May block, but must return
    // (0 or < 0) once readersCancelled() turns true or cancelBlockedReaders() was called.
    virtual long readSome(OutputStream stream, char* buffer, std::size_t capacity) = 0;
    // Blocks until the child ended and describes how (killedByUs is filled in by the base).
    virtual ExitInfo waitForChild() = 0;
    // Called when the readers outlived the child (a daemon grandchild keeps the pipes open): unblock them.
    virtual void cancelBlockedReaders() {}

    bool readersCancelled() const { return cancelReaders_; }

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool started_ = false;
    bool exited_ = false;
    std::atomic<bool> killedByUs_{false};
    std::int64_t pid_ = 0;

private:
    static constexpr int kReaderCount = 2;                    // stdout and stderr
    static constexpr std::size_t kReadChunkBytes = 8192;
    static constexpr int kReaderGraceSteps = 150;             // x 10 ms: how long the readers may outlive the child

    void readLoop(OutputStream stream);
    void monitorLoop();

    ExitInfo exit_;
    std::atomic<bool> cancelReaders_{false};
    std::atomic<int> readersDone_{0};
    ILogSink* sink_ = nullptr;
    std::thread outReader_, errReader_, monitor_;
};

}  // namespace wpiemu::core::launcher
