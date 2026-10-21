#pragma once

// Changes one process environment variable for a scope. The SDK only reads the environment, so this
// writer lives with the tests; every test executable that needs it adds tests/support as an include
// directory. A process runs its test cases one at a time (and ctest gives each discovered case its
// own process), so scopes never interleave.

#include <cstdlib>
#include <optional>
#include <string>
#include <utility>

namespace wpiemu::test_support {

// Sets, or with std::nullopt removes, `name` and restores what it was on destruction: an absent
// variable becomes absent again and an empty one empty again. The Windows CRT deletes a variable
// that is set to an empty string, so there an empty value is the same as absent.
class ScopedEnv {
public:
    ScopedEnv(std::string name, std::optional<std::string> value) : name_(std::move(name)), previous_(read(name_)) {
        write(name_, value);
    }
    ScopedEnv(const ScopedEnv&) = delete;
    ScopedEnv& operator=(const ScopedEnv&) = delete;
    ~ScopedEnv() { write(name_, previous_); }

    void set(const std::string& value) { write(name_, value); }
    void unset() { write(name_, std::nullopt); }

    // Unlike sdk::readEnvironment, an empty value is kept apart from an absent one (where the platform has both).
    static std::optional<std::string> read(const std::string& name) {
#if defined(_WIN32)
        char* buffer = nullptr;
        std::size_t length = 0;
        if (_dupenv_s(&buffer, &length, name.c_str()) != 0 || buffer == nullptr) return std::nullopt;
        std::string value = buffer;
        std::free(buffer);
        return value;
#else
        const char* raw = std::getenv(name.c_str());
        if (raw == nullptr) return std::nullopt;
        return std::string(raw);
#endif
    }

private:
    static void write(const std::string& name, const std::optional<std::string>& value) {
#if defined(_WIN32)
        _putenv_s(name.c_str(), value ? value->c_str() : "");
#else
        if (value) setenv(name.c_str(), value->c_str(), 1);
        else unsetenv(name.c_str());
#endif
    }

    std::string name_;
    std::optional<std::string> previous_;
};

}  // namespace wpiemu::test_support
