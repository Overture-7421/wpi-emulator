#pragma once

#include <string>

namespace wpiemu::sdk {

enum class Severity { Info, Warning, Error };

// A problem found while validating config / mappings; `path` is dotted ("mechanisms[2].slots.motor").
// `code` (SDK 1.1) is a stable machine key (see config/validation_codes.h); empty for legacy issues.
struct Issue {
    Severity severity = Severity::Error;
    std::string path;
    std::string message;
    std::string code;
};

}  // namespace wpiemu::sdk
