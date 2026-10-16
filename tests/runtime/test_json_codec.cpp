#include <doctest/doctest.h>

#include <cmath>
#include <limits>

#include "wpiemu/core/config/json_codec.h"

using wpiemu::core::config::floatToDouble;
using wpiemu::core::config::formatDouble;
using wpiemu::core::config::JsonValueCodec;
using wpiemu::sdk::ErrorCode;
using wpiemu::sdk::Value;

namespace {
Value parseOk(const char* text) {
    const auto r = JsonValueCodec{}.parse(text);
    REQUIRE_MESSAGE(r.ok(), (r.ok() ? "" : r.error().message));
    return r.value();
}
}  // namespace

TEST_CASE("json: scalars and number types stay distinct") {
    CHECK(parseOk("null").isNull());
    CHECK(parseOk("true").asBool() == true);
    CHECK(parseOk("false").isBool());
    CHECK(parseOk("42").isInt());
    CHECK(parseOk("-7").asInt() == -7);
    CHECK(parseOk("4.0").isDouble());
    CHECK(parseOk("1e3").isDouble());
    CHECK(parseOk("1e3").asDouble() == 1000.0);
    CHECK(parseOk("-0.5").asDouble() == -0.5);
    CHECK(parseOk("9223372036854775807").asInt() == INT64_MAX);
    CHECK(parseOk("9223372036854775808").isDouble());  // beyond int64 -> double
    CHECK(parseOk("\"hi\"").asString() == "hi");
}

TEST_CASE("json: objects keep insertion order and the last duplicate wins") {
    const char* text = R"json({"b":1,"a":2,"b":3})json";
    const Value v = parseOk(text);
    REQUIRE(v.isObject());
    CHECK(v.object().size() == 2);
    CHECK(v.object().keyAt(0) == "b");
    CHECK(v.object().keyAt(1) == "a");
    CHECK(v.find("b")->asInt() == 3);
}

TEST_CASE("json: string escapes and unicode") {
    const char* escapes = R"json("a\n\t\"\\\/b")json";
    const char* accent = R"json("\u00e9")json";
    const char* emoji = R"json("\ud83d\ude00")json";
    const char* loneHigh = R"json("\ud83d")json";
    const char* loneLow = R"json("\udc00")json";
    const char* badEscape = R"json("\x")json";
    CHECK(parseOk(escapes).asString() == "a\n\t\"\\/b");
    CHECK(parseOk(accent).asString() == "\xC3\xA9");
    CHECK(parseOk(emoji).asString() == "\xF0\x9F\x98\x80");  // surrogate pair
    CHECK(parseOk("\"\xC3\xA9\"").asString() == "\xC3\xA9");  // raw UTF-8 passes through
    CHECK_FALSE(JsonValueCodec{}.parse(loneHigh).ok());
    CHECK_FALSE(JsonValueCodec{}.parse(loneLow).ok());
    CHECK_FALSE(JsonValueCodec{}.parse(badEscape).ok());
    CHECK_FALSE(JsonValueCodec{}.parse("\"a\nb\"").ok());  // raw control character
}

TEST_CASE("json: BOM and whitespace are tolerated") {
    CHECK(parseOk("\xEF\xBB\xBF  {\"a\" : [ 1 , 2 ]}\n").find("a")->array().size() == 2);
}

TEST_CASE("json: malformed input reports a position") {
    const char* bad[] = {"",       "{",        "[1,",     "{\"a\"}",   "{\"a\":}", "[1 2]", "tru",    "nul",  "01",
                         "1.",     "1e",       "-",       "+1",        ".5",       "{'a':1}", "[1,]", "{\"a\":1,}", "1 2",
                         "\"abc", "[\"a\"",    "{\"a\":1", "nan",      "Infinity"};
    for (const char* text : bad) {
        const auto r = JsonValueCodec{}.parse(text);
        CHECK_MESSAGE(!r.ok(), text);
        if (!r.ok()) {
            CHECK(r.error().code == ErrorCode::ParseError);
            CHECK(r.error().message.find("line") != std::string::npos);
        }
    }
    const auto r = JsonValueCodec{}.parse("{\n  \"a\": ?\n}");
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().message.find("line 2") != std::string::npos);
}

TEST_CASE("json: nesting is limited instead of overflowing the stack") {
    std::string deep(500, '[');
    deep += std::string(500, ']');
    const auto r = JsonValueCodec{}.parse(deep);
    CHECK_FALSE(r.ok());
    std::string ok(100, '[');
    ok += std::string(100, ']');
    CHECK(JsonValueCodec{}.parse(ok).ok());
}

TEST_CASE("json: doubles use the shortest round-trip form") {
    CHECK(formatDouble(0.1) == "0.1");
    CHECK(formatDouble(5.0) == "5.0");
    CHECK(formatDouble(-2.5) == "-2.5");
    CHECK(formatDouble(1e22).find('e') != std::string::npos);
    CHECK(formatDouble(std::numeric_limits<double>::quiet_NaN()) == "null");
    CHECK(floatToDouble(0.7f) == 0.7);
    CHECK(floatToDouble(0.05f) == 0.05);
    CHECK(static_cast<float>(floatToDouble(0.123456789f)) == 0.123456789f);

    const double tricky[] = {0.1, 1.0 / 3.0, 2.2250738585072014e-308, 1.7976931348623157e308, 123456789.123456789, -0.0001};
    for (const double d : tricky) {
        const Value back = parseOk(JsonValueCodec{}.dump(Value(d), false).c_str());
        CHECK(back.asDouble() == d);
    }
}

TEST_CASE("json: floating-point limits and signed zero round-trip") {
    const double values[] = {0.0,
                             -0.0,
                             std::numeric_limits<double>::denorm_min(),
                             -std::numeric_limits<double>::denorm_min(),
                             std::numeric_limits<double>::min(),
                             std::numeric_limits<double>::max(),
                             std::numeric_limits<double>::lowest()};
    for (const double value : values) {
        const Value back = parseOk(formatDouble(value).c_str());
        REQUIRE(back.isDouble());
        CHECK(back.asDouble() == value);
        CHECK(std::signbit(back.asDouble()) == std::signbit(value));
    }
    CHECK_FALSE(JsonValueCodec{}.parse("1e309").ok());
    CHECK_FALSE(JsonValueCodec{}.parse("-1e309").ok());
}

TEST_CASE("json: float conversion preserves limits and signed zero") {
    const float values[] = {0.0f,
                            -0.0f,
                            std::numeric_limits<float>::denorm_min(),
                            std::numeric_limits<float>::min(),
                            std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::lowest()};
    for (const float value : values) {
        const double converted = floatToDouble(value);
        CHECK(std::isfinite(converted));
        CHECK(static_cast<float>(converted) == value);
        CHECK(std::signbit(converted) == std::signbit(value));
    }
}

TEST_CASE("json: dump compact and pretty round-trip") {
    Value v = Value::makeObject();
    v["name"] = "robot \"x\"\n";
    v["n"] = 3;
    v["d"] = 2.5;
    v["flag"] = true;
    v["none"] = nullptr;
    v["list"] = Value::makeArray();
    v["list"].push_back(1);
    v["list"].push_back("two");
    v["nested"] = Value::makeObject();
    v["nested"]["empty"] = Value::makeObject();
    v["nested"]["arr"] = Value::makeArray();
    v["ctrl"] = std::string("a\x01" "b");

    const JsonValueCodec codec;
    const std::string compact = codec.dump(v, false);
    CHECK(compact.find('\n') == std::string::npos);
    const std::string pretty = codec.dump(v, true);
    CHECK(pretty.find("\n  \"name\"") != std::string::npos);
    CHECK(pretty.find("[1, \"two\"]") != std::string::npos);  // short scalar arrays stay on one line
    CHECK(pretty.back() == '\n');
    CHECK(pretty.find("\\u0001") != std::string::npos);

    CHECK(parseOk(compact.c_str()) == v);
    CHECK(parseOk(pretty.c_str()) == v);
    // key order is stable through a round trip
    const Value back = parseOk(compact.c_str());
    CHECK(back.object().keyAt(0) == "name");
    CHECK(codec.dump(back, false) == compact);
}

TEST_CASE("json: non-finite doubles dump as null") {
    Value v = Value::makeObject();
    v["inf"] = std::numeric_limits<double>::infinity();
    CHECK(JsonValueCodec{}.dump(v, false) == "{\"inf\":null}");
}
