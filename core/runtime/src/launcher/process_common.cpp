#include <algorithm>
#include <cctype>
#include <map>

#include "process_base.h"
#include "wpiemu/core/launcher/process.h"

namespace wpiemu::core::launcher {

namespace fs = std::filesystem;

using sdk::ErrorCode;
using sdk::makeError;
using sdk::okStatus;
using sdk::Status;

bool ProcessBase::running() const {
    std::lock_guard lock(mutex_);
    return started_ && !exited_;
}

std::int64_t ProcessBase::pid() const {
    std::lock_guard lock(mutex_);
    return pid_;
}

std::optional<ExitInfo> ProcessBase::exitInfo() const {
    std::lock_guard lock(mutex_);
    return exited_ ? std::optional<ExitInfo>(exit_) : std::nullopt;
}

bool ProcessBase::waitForExit(std::chrono::milliseconds timeout) {
    std::unique_lock lock(mutex_);
    if (!started_) return true;
    return cv_.wait_for(lock, timeout, [this] { return exited_; });
}

Status ProcessBase::beginStart(ILogSink& sink) {
    if (started_) return makeError(ErrorCode::InvalidState, "process already started");
    if (killedByUs_) return makeError(ErrorCode::Cancelled, "process cancelled before startup");
    sink_ = &sink;
    return okStatus();
}

void ProcessBase::startSupervision(std::int64_t pid) {
    pid_ = pid;
    started_ = true;
    outReader_ = std::thread([this] { readLoop(OutputStream::Stdout); });
    errReader_ = std::thread([this] { readLoop(OutputStream::Stderr); });
    monitor_ = std::thread([this] { monitorLoop(); });
}

void ProcessBase::joinAll() {
    if (outReader_.joinable()) outReader_.join();
    if (errReader_.joinable()) errReader_.join();
    if (monitor_.joinable()) monitor_.join();
}

void ProcessBase::readLoop(OutputStream stream) {
    LineSplitter splitter;
    char buffer[kReadChunkBytes];
    const auto emit = [&](std::string_view line) { sink_->line(stream, line); };
    for (;;) {
        const long n = readSome(stream, buffer, sizeof(buffer));
        if (n <= 0) break;
        splitter.feed(std::string_view(buffer, static_cast<std::size_t>(n)), emit);
    }
    splitter.flush(emit);
    ++readersDone_;
}

void ProcessBase::monitorLoop() {
    ExitInfo info = waitForChild();
    // A grandchild (a daemon) may keep the pipes open: give the readers a moment, then make them stop.
    for (int i = 0; i < kReaderGraceSteps && readersDone_ < kReaderCount; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    cancelReaders_ = true;
    if (readersDone_ < kReaderCount) cancelBlockedReaders();
    {
        std::lock_guard lock(mutex_);
        info.killedByUs = killedByUs_;
        exit_ = info;
        exited_ = true;
    }
    cv_.notify_all();
}

void LineSplitter::feed(std::string_view chunk, const std::function<void(std::string_view)>& onLine) {
    for (const char c : chunk) {
        if (c == '\n') {
            if (buffer_.empty() && justCut_) {  // the newline right after a forced cut does not start a new (empty) line
                justCut_ = false;
                continue;
            }
            std::string_view line = buffer_;
            if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
            onLine(line);
            buffer_.clear();
            justCut_ = false;
            continue;
        }
        justCut_ = false;
        buffer_.push_back(c);
        if (buffer_.size() >= maxLine_) {
            onLine(buffer_);
            buffer_.clear();
            justCut_ = true;
        }
    }
}

void LineSplitter::flush(const std::function<void(std::string_view)>& onLine) {
    if (buffer_.empty()) return;
    std::string_view line = buffer_;
    if (line.back() == '\r') line.remove_suffix(1);
    onLine(line);
    buffer_.clear();
}

namespace {
std::string fold(const std::string& name, bool caseInsensitive) {
    if (!caseInsensitive) return name;
    std::string out = name;
    for (char& c : out) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return out;
}

// Index of the '=' between name and value; Windows has "=C:=C:\dir" entries whose name starts with '='.
std::size_t environmentSeparator(const std::string& kv) { return kv.find('=', kv.empty() || kv[0] != '=' ? 0 : 1); }
}  // namespace

std::vector<std::string> mergeEnvironment(const std::vector<std::string>& base, const ProcessSpec& spec, bool caseInsensitiveNames) {
    std::map<std::string, std::pair<std::string, std::string>> entries;  // folded name -> (name, value)
    for (const std::string& kv : base) {
        const std::size_t eq = environmentSeparator(kv);
        if (eq == std::string::npos || eq == 0) continue;
        const std::string name = kv.substr(0, eq);
        entries[fold(name, caseInsensitiveNames)] = {name, kv.substr(eq + 1)};
    }
    for (const std::string& name : spec.unsetEnv) entries.erase(fold(name, caseInsensitiveNames));
    for (const auto& [name, value] : spec.env) entries[fold(name, caseInsensitiveNames)] = {name, value};
    std::vector<std::string> out;
    out.reserve(entries.size());
    for (const auto& [key, nv] : entries) out.push_back(nv.first + "=" + nv.second);
    return out;
}

std::optional<std::string> lookupEnvironment(const std::vector<std::string>& env, std::string_view name, bool caseInsensitiveNames) {
    const std::string wanted = fold(std::string(name), caseInsensitiveNames);
    for (const std::string& kv : env) {
        const std::size_t eq = environmentSeparator(kv);
        if (eq == std::string::npos || eq == 0) continue;
        if (fold(kv.substr(0, eq), caseInsensitiveNames) == wanted) return kv.substr(eq + 1);
    }
    return std::nullopt;
}

std::string quoteWindowsArgument(std::string_view arg) {
    if (!arg.empty() && arg.find_first_of(" \t\n\v\"") == std::string_view::npos) return std::string(arg);
    std::string out = "\"";
    std::size_t backslashes = 0;
    for (const char c : arg) {
        if (c == '\\') {
            ++backslashes;
        } else if (c == '"') {
            out.append(backslashes * 2 + 1, '\\');
            out.push_back('"');
            backslashes = 0;
        } else {
            out.append(backslashes, '\\');
            out.push_back(c);
            backslashes = 0;
        }
    }
    out.append(backslashes * 2, '\\');  // backslashes before the closing quote must be doubled
    out.push_back('"');
    return out;
}

bool isSafeForBatchFile(std::string_view arg) { return arg.find_first_of("\"%\r\n") == std::string_view::npos && arg.find('\0') == std::string_view::npos; }

std::string quoteBatchArgument(std::string_view arg) {
    if (!arg.empty() && arg.find_first_of(" \t\n\v\"&|<>^()!,;=") == std::string_view::npos) return std::string(arg);
    std::string out = "\"";
    out.append(arg);
    std::size_t trailing = 0;
    while (trailing < arg.size() && arg[arg.size() - 1 - trailing] == '\\') ++trailing;
    out.append(trailing, '\\');  // doubled as in quoteWindowsArgument: batch files usually forward arguments to a CRT-parsed program
    out.push_back('"');
    return out;
}

std::optional<fs::path> findOnPath(std::string_view program, std::string_view pathVar, bool windows, std::string_view pathExt) {
    if (program.empty()) return std::nullopt;
    const char listSep = windows ? ';' : ':';
    auto exts = [&](const fs::path& p) {
        std::vector<fs::path> candidates;
        if (!windows || p.has_extension()) {
            candidates.push_back(p);
            return candidates;
        }
        std::size_t start = 0;
        while (start <= pathExt.size()) {
            const std::size_t end = pathExt.find(';', start);
            const std::string_view ext = pathExt.substr(start, end == std::string_view::npos ? std::string_view::npos : end - start);
            if (!ext.empty()) {
                fs::path candidate = p;
                candidate += std::string(ext);
                candidates.push_back(candidate);
            }
            if (end == std::string_view::npos) break;
            start = end + 1;
        }
        return candidates;
    };
    auto isExecutableFile = [&](const fs::path& p) {
        std::error_code ec;
        if (!fs::is_regular_file(p, ec) || ec) return false;
        if (windows) return true;
        const fs::perms perms = fs::status(p, ec).permissions();
        return !ec && (perms & (fs::perms::owner_exec | fs::perms::group_exec | fs::perms::others_exec)) != fs::perms::none;
    };

    const fs::path prog{std::string(program)};
    if (program.find('/') != std::string_view::npos || program.find('\\') != std::string_view::npos) {
        for (const fs::path& c : exts(prog))
            if (isExecutableFile(c)) return c;
        return std::nullopt;
    }
    std::size_t start = 0;
    while (start <= pathVar.size()) {
        const std::size_t end = pathVar.find(listSep, start);
        std::string_view dir = pathVar.substr(start, end == std::string_view::npos ? std::string_view::npos : end - start);
        if (windows && dir.size() >= 2 && dir.front() == '"' && dir.back() == '"') dir = dir.substr(1, dir.size() - 2);
        if (!dir.empty())
            for (const fs::path& c : exts(fs::path(std::string(dir)) / prog))
                if (isExecutableFile(c)) return c;
        if (end == std::string_view::npos) break;
        start = end + 1;
    }
    return std::nullopt;
}

ProcessFactory systemProcessFactory() {
    return [] { return createSystemProcess(); };
}

}  // namespace wpiemu::core::launcher
