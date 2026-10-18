#include "wpiemu/bridges/vision/photon_codec.h"

#include <bit>
#include <limits>
#include <map>
#include <optional>
#include <utility>

#include "wpiemu/bridges/vision/photon_md5.h"

namespace wpiemu::bridges::vision {

namespace {

using sdk::ErrorCode;
using sdk::makeError;

enum class Primitive : std::uint8_t { Bool, Int16, Int32, Int64, Float32, Float64 };

// Primitive type names of a photon schema (PhotonVision photon-serde message_data_types.yaml, v2026.3.4).
std::optional<Primitive> primitiveFor(std::string_view name) {
    if (name == "bool") return Primitive::Bool;
    if (name == "int16") return Primitive::Int16;
    if (name == "int32") return Primitive::Int32;
    if (name == "int64") return Primitive::Int64;
    if (name == "float32") return Primitive::Float32;
    if (name == "float64") return Primitive::Float64;
    return std::nullopt;
}

// WPILib structs a photon message can embed, in the same grammar as a profile text. These are WPILib's own struct
// definitions (Quaternion, Translation3d, Rotation3d, Transform3d); PhotonLib's Packet writes a Transform3d as
// exactly this concatenation (golden fixtures, tests/bridges_vision/fixtures/photon).
struct WpilibStruct {
    std::string_view name;
    std::string_view schema;
};
constexpr WpilibStruct kWpilibStructs[] = {
    {"Quaternion", "float64 w;float64 x;float64 y;float64 z;"},
    {"Translation3d", "float64 x;float64 y;float64 z;"},
    {"Rotation3d", "Quaternion q;"},
    {"Transform3d", "Translation3d translation;Rotation3d rotation;"},
};

struct FieldDef {
    std::string name;
    bool optional = false;
    bool list = false;
    std::optional<Primitive> primitive;  // empty: a nested message
    std::size_t message = 0;             // index into Impl::schemas when it is a nested message
};

struct SchemaDef {
    std::string name;
    std::string uuid;  // empty for WPILib structs
    std::vector<FieldDef> fields;
};

struct RawField {
    std::string_view type;
    std::string_view uuid;
    std::string_view name;
    bool optional = false;
    bool list = false;
};

bool isIdentifier(std::string_view text) {
    if (text.empty()) return false;
    for (std::size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        const bool letter = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
        if (!(letter || (i > 0 && c >= '0' && c <= '9'))) return false;
    }
    return true;
}

bool isUuid(std::string_view text) {
    if (text.size() != 32) return false;
    for (const char c : text) {
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    }
    return true;
}

std::vector<std::string_view> splitWords(std::string_view text) {
    std::vector<std::string_view> words;
    std::size_t i = 0;
    while (i < text.size()) {
        while (i < text.size() && (text[i] == ' ' || text[i] == '\t' || text[i] == '\n' || text[i] == '\r')) ++i;
        const std::size_t start = i;
        while (i < text.size() && !(text[i] == ' ' || text[i] == '\t' || text[i] == '\n' || text[i] == '\r')) ++i;
        if (i > start) words.push_back(text.substr(start, i - start));
    }
    return words;
}

// "<type>[:<uuid>] <name>[[?]];" fields, optionally prefixed by "optional ".
sdk::Result<std::vector<RawField>> parseSchemaText(std::string_view message, std::string_view text) {
    const auto fail = [&](std::string_view piece, std::string_view why) {
        return makeError(ErrorCode::ParseError,
                         std::string(message) + ": cannot parse '" + std::string(piece) + "': " + std::string(why));
    };
    std::vector<RawField> fields;
    std::size_t start = 0;
    while (start < text.size()) {
        const std::size_t end = text.find(';', start);
        if (end == std::string_view::npos) return fail(text.substr(start), "a field must end with ';'");
        const std::string_view piece = text.substr(start, end - start);
        start = end + 1;

        std::vector<std::string_view> words = splitWords(piece);
        RawField field;
        if (!words.empty() && words.front() == "optional") {
            field.optional = true;
            words.erase(words.begin());
        }
        if (words.size() != 2) return fail(piece, "expected '<type> <name>'");

        std::string_view type = words[0];
        const std::size_t colon = type.find(':');
        if (colon != std::string_view::npos) {
            field.uuid = type.substr(colon + 1);
            type = type.substr(0, colon);
            if (!isUuid(field.uuid)) return fail(piece, "the UUID after ':' must be 32 lower-case hex digits");
        }
        if (!isIdentifier(type)) return fail(piece, "bad type name");
        field.type = type;

        std::string_view name = words[1];
        if (name.size() > 3 && name.substr(name.size() - 3) == "[?]") {
            field.list = true;
            name.remove_suffix(3);
        } else if (name.find('[') != std::string_view::npos) {
            return fail(piece, "only variable length lists, name[?], are supported");
        }
        if (!isIdentifier(name)) return fail(piece, "bad field name");
        field.name = name;
        if (field.optional && field.list) return fail(piece, "a field cannot be both optional and a list");
        fields.push_back(field);
    }
    return fields;
}

}  // namespace

struct PhotonCodec::Impl {
    std::string uuid;
    std::string version;
    std::vector<SchemaDef> schemas;
    std::map<std::string, std::size_t, std::less<>> byName;
};

namespace {

// ---- encoding ---------------------------------------------------------------------------------------------

// Where in the value tree an error happened, for its message: "targets[2].bestCameraToTarget.translation.x".
class Path {
public:
    void pushName(std::string_view name) { segments_.push_back({name, 0, false}); }
    void pushIndex(std::size_t index) { segments_.push_back({{}, index, true}); }
    void pop() { segments_.pop_back(); }

    std::string str() const {
        std::string text;
        for (const Segment& s : segments_) {
            if (s.isIndex) {
                text += '[' + std::to_string(s.index) + ']';
            } else {
                if (!text.empty()) text += '.';
                text += s.name;
            }
        }
        return text.empty() ? std::string("(root)") : text;
    }

private:
    struct Segment {
        std::string_view name;
        std::size_t index;
        bool isIndex;
    };
    std::vector<Segment> segments_;
};

class Encoder {
public:
    Encoder(const PhotonCodec::Impl& impl, std::string_view message, std::vector<std::uint8_t>& out)
        : impl_(impl), message_(message), out_(out) {}

    sdk::Status message(const SchemaDef& schema, const PhotonValue& value) {
        if (value.kind() != PhotonValue::Kind::Struct) return fail("expected a struct for " + schema.name);
        for (const FieldDef& field : schema.fields) {
            path_.pushName(field.name);
            const PhotonValue* member = value.find(field.name);
            if (member == nullptr) return fail("no value for the schema field " + schema.name + "." + field.name);
            if (sdk::Status status = encodeField(field, *member); !status.ok()) return status;
            path_.pop();
        }
        return sdk::okStatus();
    }

private:
    sdk::Status fail(const std::string& what) const {
        return makeError(ErrorCode::InvalidArgument, std::string(message_) + ": " + path_.str() + ": " + what);
    }

    void put(std::uint64_t bits, std::size_t bytes) {
        for (std::size_t i = 0; i < bytes; ++i) out_.push_back(static_cast<std::uint8_t>((bits >> (8U * i)) & 0xffU));
    }

    sdk::Status encodeField(const FieldDef& field, const PhotonValue& value) {
        if (field.optional) {
            if (value.isAbsent()) {
                put(0, 1);
                return sdk::okStatus();
            }
            put(1, 1);
            return encodeElement(field, value);
        }
        if (!field.list) return encodeElement(field, value);

        if (value.kind() != PhotonValue::Kind::List) return fail("expected a list");
        const std::vector<PhotonValue>& items = value.children();
        if (items.size() > kPhotonMaxListLength) {
            return fail("the list has " + std::to_string(items.size()) + " items, PhotonLib allows at most " +
                        std::to_string(kPhotonMaxListLength));
        }
        put(items.size(), 1);
        for (std::size_t i = 0; i < items.size(); ++i) {
            path_.pushIndex(i);
            if (sdk::Status status = encodeElement(field, items[i]); !status.ok()) return status;
            path_.pop();
        }
        return sdk::okStatus();
    }

    sdk::Status encodeElement(const FieldDef& field, const PhotonValue& value) {
        if (!field.primitive) return message(impl_.schemas[field.message], value);
        switch (*field.primitive) {
            case Primitive::Bool:
                if (value.kind() != PhotonValue::Kind::Bool) return fail("expected a bool");
                put(value.boolValue() ? 1 : 0, 1);
                return sdk::okStatus();
            case Primitive::Int16:
                return encodeInteger(value, std::numeric_limits<std::int16_t>::min(),
                                     std::numeric_limits<std::int16_t>::max(), 2);
            case Primitive::Int32:
                return encodeInteger(value, std::numeric_limits<std::int32_t>::min(),
                                     std::numeric_limits<std::int32_t>::max(), 4);
            case Primitive::Int64:
                return encodeInteger(value, std::numeric_limits<std::int64_t>::min(),
                                     std::numeric_limits<std::int64_t>::max(), 8);
            case Primitive::Float32:
                if (!isNumber(value)) return fail("expected a number");
                put(std::bit_cast<std::uint32_t>(static_cast<float>(value.realValue())), 4);
                return sdk::okStatus();
            case Primitive::Float64:
                if (!isNumber(value)) return fail("expected a number");
                put(std::bit_cast<std::uint64_t>(value.realValue()), 8);
                return sdk::okStatus();
        }
        return fail("unknown primitive type");
    }

    static bool isNumber(const PhotonValue& value) {
        return value.kind() == PhotonValue::Kind::Real || value.kind() == PhotonValue::Kind::Int;
    }

    sdk::Status encodeInteger(const PhotonValue& value, std::int64_t min, std::int64_t max, std::size_t bytes) {
        if (value.kind() != PhotonValue::Kind::Int) return fail("expected an integer");
        if (value.intValue() < min || value.intValue() > max) {
            return fail("the integer " + std::to_string(value.intValue()) + " does not fit " +
                        std::to_string(bytes * 8) + " bits");
        }
        put(static_cast<std::uint64_t>(value.intValue()), bytes);
        return sdk::okStatus();
    }

    const PhotonCodec::Impl& impl_;
    std::string_view message_;
    std::vector<std::uint8_t>& out_;
    Path path_;
};

// ---- decoding ---------------------------------------------------------------------------------------------

class Decoder {
public:
    Decoder(const PhotonCodec::Impl& impl, std::string_view message, std::span<const std::uint8_t> bytes)
        : impl_(impl), message_(message), data_(bytes) {}

    sdk::Result<PhotonValue> message(const SchemaDef& schema) {
        PhotonValue result = PhotonValue::structure();
        for (const FieldDef& field : schema.fields) {
            path_.pushName(field.name);
            sdk::Result<PhotonValue> member = decodeField(field);
            if (!member.ok()) return member.error();
            result.set(field.name, std::move(member).value());
            path_.pop();
        }
        return result;
    }

    sdk::Status finish() const {
        if (pos_ != data_.size()) return failStatus(std::to_string(data_.size() - pos_) + " trailing bytes");
        return sdk::okStatus();
    }

private:
    sdk::Error failError(const std::string& what) const {
        return makeError(ErrorCode::ProtocolError,
                         std::string(message_) + ": " + path_.str() + " at byte " + std::to_string(pos_) + ": " + what);
    }
    sdk::Status failStatus(const std::string& what) const { return failError(what); }

    bool take(std::size_t bytes, std::uint64_t& value) {
        if (data_.size() - pos_ < bytes) return false;
        value = 0;
        for (std::size_t i = 0; i < bytes; ++i) value |= static_cast<std::uint64_t>(data_[pos_ + i]) << (8U * i);
        pos_ += bytes;
        return true;
    }

    sdk::Result<PhotonValue> decodeField(const FieldDef& field) {
        std::uint64_t raw = 0;
        if (field.optional) {
            if (!take(1, raw)) return failError("the message ends before an optional flag");
            if (raw > 1) return failError("an optional flag must be 0 or 1");
            if (raw == 0) return PhotonValue::absent();
            return decodeElement(field);
        }
        if (!field.list) return decodeElement(field);

        if (!take(1, raw)) return failError("the message ends before a list length");
        if (raw > kPhotonMaxListLength) {
            return failError("list length " + std::to_string(raw) + " is above " +
                             std::to_string(kPhotonMaxListLength));
        }
        PhotonValue list = PhotonValue::list();
        for (std::uint64_t i = 0; i < raw; ++i) {
            path_.pushIndex(static_cast<std::size_t>(i));
            sdk::Result<PhotonValue> item = decodeElement(field);
            if (!item.ok()) return item.error();
            list.push(std::move(item).value());
            path_.pop();
        }
        return list;
    }

    sdk::Result<PhotonValue> decodeElement(const FieldDef& field) {
        if (!field.primitive) return message(impl_.schemas[field.message]);
        std::uint64_t raw = 0;
        switch (*field.primitive) {
            case Primitive::Bool:
                if (!take(1, raw)) return truncated();
                if (raw > 1) return failError("a bool must be 0 or 1");
                return PhotonValue::boolean(raw == 1);
            case Primitive::Int16:
                if (!take(2, raw)) return truncated();
                return PhotonValue::integer(static_cast<std::int16_t>(static_cast<std::uint16_t>(raw)));
            case Primitive::Int32:
                if (!take(4, raw)) return truncated();
                return PhotonValue::integer(static_cast<std::int32_t>(static_cast<std::uint32_t>(raw)));
            case Primitive::Int64:
                if (!take(8, raw)) return truncated();
                return PhotonValue::integer(static_cast<std::int64_t>(raw));
            case Primitive::Float32:
                if (!take(4, raw)) return truncated();
                return PhotonValue::real(static_cast<double>(std::bit_cast<float>(static_cast<std::uint32_t>(raw))));
            case Primitive::Float64:
                if (!take(8, raw)) return truncated();
                return PhotonValue::real(std::bit_cast<double>(raw));
        }
        return failError("unknown primitive type");
    }

    sdk::Error truncated() const { return failError("the message is truncated"); }

    const PhotonCodec::Impl& impl_;
    std::string_view message_;
    std::span<const std::uint8_t> data_;
    std::size_t pos_ = 0;
    Path path_;
};

}  // namespace

sdk::Result<PhotonCodec> PhotonCodec::create(const PhotonProfile& profile) {
    auto impl = std::make_shared<Impl>();
    impl->uuid = std::string(profile.uuid);
    impl->version = std::string(profile.version);
    if (profile.messages.empty())
        return makeError(ErrorCode::InvalidArgument, "the PhotonVision profile has no messages");
    if (profile.version.empty()) {
        return makeError(ErrorCode::InvalidArgument,
                         "the PhotonVision profile has no version text (PhotonLib's C++ camera takes an empty "
                         "/photonvision/version for \"no coprocessor\")");
    }

    // Pass 1: register every message. Photon messages first (their text must hash to their UUID), then the WPILib
    // structs they may embed.
    std::vector<std::string_view> texts;
    for (const PhotonMessageDef& def : profile.messages) {
        const std::string name(def.name);
        if (!isIdentifier(def.name)) return makeError(ErrorCode::InvalidArgument, "bad message name '" + name + "'");
        if (impl->byName.contains(name)) {
            return makeError(ErrorCode::InvalidArgument, "message " + name + " is defined twice");
        }
        for (const WpilibStruct& wpilib : kWpilibStructs) {
            if (wpilib.name == def.name) {
                return makeError(ErrorCode::InvalidArgument, "message " + name + " clashes with the WPILib struct");
            }
        }
        if (!isUuid(def.uuid)) {
            return makeError(ErrorCode::InvalidArgument,
                             "message " + name + " has no valid UUID (32 lower-case hex digits)");
        }
        const std::string digest = md5Hex(def.schema);
        if (digest != def.uuid) {
            return makeError(ErrorCode::InvalidArgument, "message " + name + ": the MD5 of its schema text is " +
                                                             digest + " but the profile says " + std::string(def.uuid));
        }
        impl->byName.emplace(name, impl->schemas.size());
        impl->schemas.push_back(SchemaDef{name, std::string(def.uuid), {}});
        texts.push_back(def.schema);
    }
    for (const WpilibStruct& wpilib : kWpilibStructs) {
        impl->byName.emplace(std::string(wpilib.name), impl->schemas.size());
        impl->schemas.push_back(SchemaDef{std::string(wpilib.name), {}, {}});
        texts.push_back(wpilib.schema);
    }

    // Pass 2: parse every text and resolve its types.
    for (std::size_t i = 0; i < impl->schemas.size(); ++i) {
        SchemaDef& schema = impl->schemas[i];
        sdk::Result<std::vector<RawField>> raw = parseSchemaText(schema.name, texts[i]);
        if (!raw.ok()) return raw.error();
        for (const RawField& rawField : raw.value()) {
            const std::string where = schema.name + "." + std::string(rawField.name);
            FieldDef field;
            field.name = std::string(rawField.name);
            field.optional = rawField.optional;
            field.list = rawField.list;
            field.primitive = primitiveFor(rawField.type);
            if (field.primitive) {
                if (!rawField.uuid.empty()) {
                    return makeError(ErrorCode::InvalidArgument, where + ": a primitive type cannot carry a UUID");
                }
            } else {
                const auto it = impl->byName.find(rawField.type);
                if (it == impl->byName.end()) {
                    if (!rawField.uuid.empty()) {
                        return makeError(ErrorCode::InvalidArgument,
                                         where + ": refers to " + std::string(rawField.type) + ":" +
                                             std::string(rawField.uuid) + ", which the profile does not define");
                    }
                    return makeError(ErrorCode::Unsupported,
                                     where + ": unknown type '" + std::string(rawField.type) + "'");
                }
                const SchemaDef& target = impl->schemas[it->second];
                if (target.uuid.empty() != rawField.uuid.empty() ||
                    (!target.uuid.empty() && target.uuid != rawField.uuid)) {
                    return makeError(ErrorCode::InvalidArgument, where + ": refers to " + std::string(rawField.type) +
                                                                     ":" + std::string(rawField.uuid) +
                                                                     " but the profile defines " + target.name + ":" +
                                                                     target.uuid);
                }
                field.message = it->second;
            }
            schema.fields.push_back(std::move(field));
        }
    }

    // The root. (No check for a message that contains itself: a text would have to contain the MD5 of itself, and
    // every text here has just been checked against its MD5; the WPILib structs are fixed.)
    const auto root = impl->byName.find(kPhotonResultMessage);
    if (root == impl->byName.end() || impl->schemas[root->second].uuid != profile.uuid) {
        return makeError(ErrorCode::InvalidArgument, "the profile has no " + std::string(kPhotonResultMessage) +
                                                         " message with UUID " + std::string(profile.uuid));
    }
    return PhotonCodec(std::move(impl));
}

const std::string& PhotonCodec::uuid() const {
    return impl_->uuid;
}

const std::string& PhotonCodec::version() const {
    return impl_->version;
}

std::string PhotonCodec::resultTypeString() const {
    return photonTypeString(kPhotonResultMessage, impl_->uuid);
}

sdk::Status PhotonCodec::encode(std::string_view message, const PhotonValue& value,
                                std::vector<std::uint8_t>& out) const {
    out.clear();
    const auto it = impl_->byName.find(message);
    if (it == impl_->byName.end())
        return makeError(ErrorCode::NotFound, "no message " + std::string(message) + " in the profile");
    Encoder encoder(*impl_, message, out);
    sdk::Status status = encoder.message(impl_->schemas[it->second], value);
    if (!status.ok()) out.clear();
    return status;
}

sdk::Result<PhotonValue> PhotonCodec::decode(std::string_view message, std::span<const std::uint8_t> bytes) const {
    const auto it = impl_->byName.find(message);
    if (it == impl_->byName.end())
        return makeError(ErrorCode::NotFound, "no message " + std::string(message) + " in the profile");
    Decoder decoder(*impl_, message, bytes);
    sdk::Result<PhotonValue> value = decoder.message(impl_->schemas[it->second]);
    if (!value.ok()) return value;
    if (sdk::Status status = decoder.finish(); !status.ok()) return status.error();
    return value;
}

}  // namespace wpiemu::bridges::vision
