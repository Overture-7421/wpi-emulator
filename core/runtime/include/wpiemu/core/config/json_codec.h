#pragma once

#include <string>
#include <string_view>

#include "wpiemu/sdk/config/config_io.h"

namespace wpiemu::core::config {

// In-house JSON <-> sdk::Value codec (ADR 0010: no third-party JSON dependency, Value objects keep
// insertion order so files are diff-friendly). RFC 8259 grammar; UTF-8 text; a leading BOM is skipped.
//  * int64 and double stay distinct: a number without fraction/exponent that fits int64 parses as int;
//    doubles are written in the shortest form that round-trips and always keep a '.'/exponent.
//  * NaN / infinity are not representable in JSON: dump() writes null.
//  * Nesting is limited (kMaxDepth) so hostile files cannot overflow the stack.
//  * Duplicate object keys: the last one wins.
class JsonValueCodec final : public sdk::IValueTextCodec {
public:
    static constexpr int kMaxDepth = 128;

    sdk::Result<sdk::Value> parse(std::string_view text) const override;
    std::string dump(const sdk::Value& value, bool pretty) const override;
};

// Shortest decimal text of `d` that parses back to exactly `d` ("0.1", "1e+22", "5.0"); locale independent.
std::string formatDouble(double d);
// The double nearest to the shortest decimal that round-trips the float, e.g. 0.7f -> 0.7 (not 0.699999988...).
double floatToDouble(float f);

}  // namespace wpiemu::core::config
