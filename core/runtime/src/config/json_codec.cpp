#include "wpiemu/core/config/json_codec.h"

#include <charconv>
#include <cmath>
#include <cstdint>
#include <exception>
#include <locale>
#include <sstream>
#include <system_error>

namespace wpiemu::core::config {

using sdk::ErrorCode;
using sdk::Value;

namespace {

bool parseDouble(std::string_view text, double& out) {
#if WPIEMU_HAS_FLOAT_CHARCONV
    const auto [ptr, ec] = std::from_chars(text.data(), text.data() + text.size(), out);
    return ec == std::errc{} && ptr == text.data() + text.size();
#else
    std::istringstream in{std::string(text)};
    in.imbue(std::locale::classic());
    in >> out;
    return !in.fail() && in.eof();
#endif
}

std::string formatShortest(double d) {
#if WPIEMU_HAS_FLOAT_CHARCONV
    char buf[64];
    const auto res = std::to_chars(buf, buf + sizeof(buf), d);
    return std::string(buf, res.ptr);
#else
    for (int precision = 1; precision <= 17; ++precision) {
        std::ostringstream out;
        out.imbue(std::locale::classic());
        out.precision(precision);
        out << d;
        double back = 0.0;
        if (parseDouble(out.str(), back) && back == d) return out.str();
    }
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out.precision(17);
    out << d;
    return out.str();
#endif
}

class Parser {
public:
    explicit Parser(std::string_view text) : s_(text) {}

    sdk::Result<Value> run() {
        if (s_.size() >= 3 && static_cast<unsigned char>(s_[0]) == 0xEF && static_cast<unsigned char>(s_[1]) == 0xBB &&
            static_cast<unsigned char>(s_[2]) == 0xBF)
            pos_ = 3;
        skipWs();
        Value v;
        if (!parseValue(v, 0)) return sdk::makeError(ErrorCode::ParseError, error_);
        skipWs();
        if (pos_ != s_.size()) return fail("unexpected trailing characters");
        return v;
    }

private:
    sdk::Error fail(const std::string& what) {
        setError(what);
        return sdk::makeError(ErrorCode::ParseError, error_);
    }
    bool setError(const std::string& what) {
        if (!error_.empty()) return false;
        std::size_t line = 1, col = 1;
        for (std::size_t i = 0; i < pos_ && i < s_.size(); ++i) {
            if (s_[i] == '\n') {
                ++line;
                col = 1;
            } else {
                ++col;
            }
        }
        error_ = "JSON parse error at line " + std::to_string(line) + ", column " + std::to_string(col) + ": " + what;
        return false;
    }

    void skipWs() {
        while (pos_ < s_.size() && (s_[pos_] == ' ' || s_[pos_] == '\t' || s_[pos_] == '\n' || s_[pos_] == '\r')) ++pos_;
    }
    bool atEnd() const { return pos_ >= s_.size(); }

    bool consumeLiteral(std::string_view lit) {
        if (s_.substr(pos_, lit.size()) != lit) return setError("invalid literal");
        pos_ += lit.size();
        return true;
    }

    bool parseValue(Value& out, int depth) {
        if (depth > JsonValueCodec::kMaxDepth) return setError("nesting too deep");
        if (atEnd()) return setError("unexpected end of input");
        switch (s_[pos_]) {
            case 'n':
                out = Value{};
                return consumeLiteral("null");
            case 't':
                out = Value(true);
                return consumeLiteral("true");
            case 'f':
                out = Value(false);
                return consumeLiteral("false");
            case '"': {
                std::string str;
                if (!parseString(str)) return false;
                out = Value(std::move(str));
                return true;
            }
            case '[': return parseArray(out, depth);
            case '{': return parseObject(out, depth);
            default: return parseNumber(out);
        }
    }

    bool parseArray(Value& out, int depth) {
        ++pos_;  // [
        Value arr = Value::makeArray();
        skipWs();
        if (!atEnd() && s_[pos_] == ']') {
            ++pos_;
            out = std::move(arr);
            return true;
        }
        for (;;) {
            skipWs();
            Value element;
            if (!parseValue(element, depth + 1)) return false;
            arr.push_back(std::move(element));
            skipWs();
            if (atEnd()) return setError("unterminated array");
            if (s_[pos_] == ',') {
                ++pos_;
                continue;
            }
            if (s_[pos_] == ']') {
                ++pos_;
                break;
            }
            return setError("expected ',' or ']'");
        }
        out = std::move(arr);
        return true;
    }

    bool parseObject(Value& out, int depth) {
        ++pos_;  // {
        Value obj = Value::makeObject();
        skipWs();
        if (!atEnd() && s_[pos_] == '}') {
            ++pos_;
            out = std::move(obj);
            return true;
        }
        for (;;) {
            skipWs();
            if (atEnd() || s_[pos_] != '"') return setError("expected string key");
            std::string key;
            if (!parseString(key)) return false;
            skipWs();
            if (atEnd() || s_[pos_] != ':') return setError("expected ':'");
            ++pos_;
            skipWs();
            Value member;
            if (!parseValue(member, depth + 1)) return false;
            obj.object().set(key, std::move(member));
            skipWs();
            if (atEnd()) return setError("unterminated object");
            if (s_[pos_] == ',') {
                ++pos_;
                continue;
            }
            if (s_[pos_] == '}') {
                ++pos_;
                break;
            }
            return setError("expected ',' or '}'");
        }
        out = std::move(obj);
        return true;
    }

    static void appendUtf8(std::string& out, std::uint32_t cp) {
        if (cp < 0x80) {
            out.push_back(static_cast<char>(cp));
        } else if (cp < 0x800) {
            out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        } else if (cp < 0x10000) {
            out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
            out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        } else {
            out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
            out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        }
    }

    bool parseHex4(std::uint32_t& out) {
        if (pos_ + 4 > s_.size()) return setError("truncated \\u escape");
        std::uint32_t v = 0;
        for (int i = 0; i < 4; ++i) {
            const char c = s_[pos_ + static_cast<std::size_t>(i)];
            v <<= 4;
            if (c >= '0' && c <= '9') v |= static_cast<std::uint32_t>(c - '0');
            else if (c >= 'a' && c <= 'f') v |= static_cast<std::uint32_t>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') v |= static_cast<std::uint32_t>(c - 'A' + 10);
            else return setError("invalid \\u escape");
        }
        pos_ += 4;
        out = v;
        return true;
    }

    bool parseString(std::string& out) {
        ++pos_;  // opening quote
        for (;;) {
            if (atEnd()) return setError("unterminated string");
            const char c = s_[pos_++];
            if (c == '"') return true;
            if (static_cast<unsigned char>(c) < 0x20) {
                --pos_;
                return setError("control character in string");
            }
            if (c != '\\') {
                out.push_back(c);
                continue;
            }
            if (atEnd()) return setError("unterminated escape");
            const char e = s_[pos_++];
            switch (e) {
                case '"': out.push_back('"'); break;
                case '\\': out.push_back('\\'); break;
                case '/': out.push_back('/'); break;
                case 'b': out.push_back('\b'); break;
                case 'f': out.push_back('\f'); break;
                case 'n': out.push_back('\n'); break;
                case 'r': out.push_back('\r'); break;
                case 't': out.push_back('\t'); break;
                case 'u': {
                    std::uint32_t cp = 0;
                    if (!parseHex4(cp)) return false;
                    if (cp >= 0xD800 && cp <= 0xDBFF) {
                        if (s_.substr(pos_, 2) != "\\u") return setError("unpaired surrogate");
                        pos_ += 2;
                        std::uint32_t low = 0;
                        if (!parseHex4(low)) return false;
                        if (low < 0xDC00 || low > 0xDFFF) return setError("invalid surrogate pair");
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (low - 0xDC00);
                    } else if (cp >= 0xDC00 && cp <= 0xDFFF) {
                        return setError("unpaired surrogate");
                    }
                    appendUtf8(out, cp);
                    break;
                }
                default:
                    --pos_;
                    return setError("invalid escape");
            }
        }
    }

    bool parseNumber(Value& out) {
        const std::size_t start = pos_;
        auto digits = [&] {
            std::size_t n = 0;
            while (!atEnd() && s_[pos_] >= '0' && s_[pos_] <= '9') {
                ++pos_;
                ++n;
            }
            return n;
        };
        if (!atEnd() && s_[pos_] == '-') ++pos_;
        if (atEnd()) return setError("invalid number");
        if (s_[pos_] == '0') {
            ++pos_;
        } else if (digits() == 0) {
            pos_ = start;
            return setError("unexpected character");
        }
        bool integral = true;
        if (!atEnd() && s_[pos_] == '.') {
            integral = false;
            ++pos_;
            if (digits() == 0) return setError("digits expected after '.'");
        }
        if (!atEnd() && (s_[pos_] == 'e' || s_[pos_] == 'E')) {
            integral = false;
            ++pos_;
            if (!atEnd() && (s_[pos_] == '+' || s_[pos_] == '-')) ++pos_;
            if (digits() == 0) return setError("digits expected in exponent");
        }
        const std::string_view text = s_.substr(start, pos_ - start);
        if (integral) {
            std::int64_t iv = 0;
            const auto [ptr, ec] = std::from_chars(text.data(), text.data() + text.size(), iv);
            if (ec == std::errc{} && ptr == text.data() + text.size()) {
                out = Value(iv);
                return true;
            }
            // beyond int64: falls through to double
        }
        double dv = 0.0;
        if (!parseDouble(text, dv) || !std::isfinite(dv)) return setError("number out of range");
        out = Value(dv);
        return true;
    }

    std::string_view s_;
    std::size_t pos_ = 0;
    std::string error_;
};

void escapeString(std::string& out, const std::string& s) {
    out.push_back('"');
    for (const char ch : s) {
        const unsigned char c = static_cast<unsigned char>(ch);
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) {
                    static constexpr char kHex[] = "0123456789abcdef";
                    out += "\\u00";
                    out.push_back(kHex[c >> 4]);
                    out.push_back(kHex[c & 0xF]);
                } else {
                    out.push_back(ch);
                }
        }
    }
    out.push_back('"');
}

bool isScalar(const Value& v) { return !v.isArray() && !v.isObject(); }

void write(std::string& out, const Value& v, bool pretty, int indent);

void writeScalar(std::string& out, const Value& v) {
    if (v.isNull()) out += "null";
    else if (v.isBool()) out += v.asBool() ? "true" : "false";
    else if (v.isInt()) out += std::to_string(v.asInt());
    else if (v.isDouble()) out += formatDouble(v.asDouble());
    else escapeString(out, v.asString());
}

void newline(std::string& out, int indent) {
    out.push_back('\n');
    out.append(static_cast<std::size_t>(indent) * 2, ' ');
}

void write(std::string& out, const Value& v, bool pretty, int indent) {
    if (isScalar(v)) {
        writeScalar(out, v);
        return;
    }
    if (v.isArray()) {
        const auto& a = v.array();
        if (a.empty()) {
            out += "[]";
            return;
        }
        bool inlineArray = pretty;
        if (pretty) {
            std::size_t width = 0;
            for (const Value& e : a) {
                if (!isScalar(e)) {
                    inlineArray = false;
                    break;
                }
                std::string tmp;
                writeScalar(tmp, e);
                width += tmp.size() + 2;
            }
            if (width > 72) inlineArray = false;
        }
        out.push_back('[');
        for (std::size_t i = 0; i < a.size(); ++i) {
            if (i > 0) out += pretty && inlineArray ? ", " : ",";
            if (pretty && !inlineArray) newline(out, indent + 1);
            write(out, a[i], pretty, indent + 1);
        }
        if (pretty && !inlineArray) newline(out, indent);
        out.push_back(']');
        return;
    }
    const auto& o = v.object();
    if (o.empty()) {
        out += "{}";
        return;
    }
    out.push_back('{');
    for (std::size_t i = 0; i < o.size(); ++i) {
        if (i > 0) out.push_back(',');
        if (pretty) newline(out, indent + 1);
        escapeString(out, o.keyAt(i));
        out += pretty ? ": " : ":";
        write(out, o.valueAt(i), pretty, indent + 1);
    }
    if (pretty) newline(out, indent);
    out.push_back('}');
}

}  // namespace

std::string formatDouble(double d) {
    if (!std::isfinite(d)) return "null";
    std::string s = formatShortest(d);
    if (s.find_first_of(".eEn") == std::string::npos) s += ".0";
    return s;
}

double floatToDouble(float f) {
    if (!std::isfinite(f)) return static_cast<double>(f);
#if WPIEMU_HAS_FLOAT_CHARCONV
    char buf[64];
    const auto res = std::to_chars(buf, buf + sizeof(buf), f);
    double d = 0.0;
    if (parseDouble(std::string_view(buf, static_cast<std::size_t>(res.ptr - buf)), d)) return d;
    return static_cast<double>(f);
#else
    for (int precision = 1; precision <= 9; ++precision) {
        std::ostringstream out;
        out.imbue(std::locale::classic());
        out.precision(precision);
        out << f;
        double d = 0.0;
        if (parseDouble(out.str(), d) && static_cast<float>(d) == f) return d;
    }
    return static_cast<double>(f);
#endif
}

sdk::Result<Value> JsonValueCodec::parse(std::string_view text) const {
    try {
        return Parser(text).run();
    } catch (const std::exception& e) {
        return sdk::makeError(ErrorCode::ParseError, std::string("JSON parse failure: ") + e.what());
    }
}

std::string JsonValueCodec::dump(const Value& value, bool pretty) const {
    std::string out;
    write(out, value, pretty, 0);
    if (pretty) out.push_back('\n');
    return out;
}

}  // namespace wpiemu::core::config
