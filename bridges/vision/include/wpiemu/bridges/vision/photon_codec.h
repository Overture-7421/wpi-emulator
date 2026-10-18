#pragma once

// The generic PhotonVision "photonstruct" codec: ONE recursive encoder (and its mirror decoder) that walks the
// schema text of a profile (photon_profile.h) over a tree of named values, so a new message version is new
// profile data and never new encoder code.
//
// Wire rules (PhotonVision photon-serde README and message_data_types.yaml, photon-lib Packet, v2026.3.4;
// cross-checked byte for byte against the real serde, tests/bridges_vision/fixtures/photon):
//   * little-endian, no padding; bool = 1 byte; int16/32/64 and float32/64 at their natural width;
//   * a variable length list "name[?]" = one unsigned byte count (at most 127) followed by the items;
//   * "optional" = one bool byte, then the payload only when it is 1;
//   * a nested message is its fields in schema order, with no header;
//   * WPILib structs are plain concatenations of their members (Transform3d = Translation3d x y z, then the
//     Rotation3d quaternion w x y z, all float64: 56 bytes).
// A field the value tree cannot provide is an error, never a default: the caller refuses the profile instead of
// guessing (photon_mapper.h verifyPhotonMapping).

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/bridges/vision/photon_profile.h"
#include "wpiemu/sdk/core/result.h"

namespace wpiemu::bridges::vision {

// PhotonLib's Packet refuses lists above 127 items (the count is one byte, Packet.MAX_ARRAY_LEN).
inline constexpr std::size_t kPhotonMaxListLength = 127;

// A tree of values shaped like a message: structs hold NAMED members (the schema's field names), lists hold
// items, leaves are a bool, an integer or a real. An optional member that is not present is Absent.
class PhotonValue {
public:
    enum class Kind : std::uint8_t { Absent, Bool, Int, Real, Struct, List };

    PhotonValue() = default;
    static PhotonValue absent() { return {}; }
    static PhotonValue boolean(bool value) {
        PhotonValue v;
        v.kind_ = Kind::Bool;
        v.int_ = value ? 1 : 0;
        return v;
    }
    static PhotonValue integer(std::int64_t value) {
        PhotonValue v;
        v.kind_ = Kind::Int;
        v.int_ = value;
        return v;
    }
    static PhotonValue real(double value) {
        PhotonValue v;
        v.kind_ = Kind::Real;
        v.real_ = value;
        return v;
    }
    static PhotonValue structure() {
        PhotonValue v;
        v.kind_ = Kind::Struct;
        return v;
    }
    static PhotonValue list() {
        PhotonValue v;
        v.kind_ = Kind::List;
        return v;
    }

    Kind kind() const { return kind_; }
    bool isAbsent() const { return kind_ == Kind::Absent; }
    bool boolValue() const { return int_ != 0; }
    std::int64_t intValue() const { return int_; }
    // The number as a double: a real as it is, an integer converted.
    double realValue() const { return kind_ == Kind::Int ? static_cast<double>(int_) : real_; }

    // Struct: appends the member `name` (names are expected to be unique). Returns *this to chain.
    PhotonValue& set(std::string name, PhotonValue value) {
        names_.push_back(std::move(name));
        children_.push_back(std::move(value));
        return *this;
    }
    // List: appends an item. Returns *this to chain.
    PhotonValue& push(PhotonValue value) {
        children_.push_back(std::move(value));
        return *this;
    }
    // Struct: the member called `name`, or nullptr.
    const PhotonValue* find(std::string_view name) const {
        for (std::size_t i = 0; i < names_.size(); ++i) {
            if (names_[i] == name) return &children_[i];
        }
        return nullptr;
    }
    // The member values of a struct (names() has the matching names) or the items of a list.
    const std::vector<PhotonValue>& children() const { return children_; }
    const std::vector<std::string>& names() const { return names_; }

    bool operator==(const PhotonValue&) const = default;

private:
    Kind kind_ = Kind::Absent;
    std::int64_t int_ = 0;  // Bool (0/1) and Int
    double real_ = 0.0;
    std::vector<std::string> names_;     // Struct only, parallel to children_
    std::vector<PhotonValue> children_;  // Struct members or List items
};

// A profile compiled for use: every message checked against its UUID, parsed and cross-resolved. Immutable and
// cheap to copy.
class PhotonCodec {
public:
    // Checks md5(schema) == uuid for every message, parses every schema, resolves every nested type (photon
    // messages by name and UUID, WPILib structs by name) and requires PhotonPipelineResult to carry the
    // profile's UUID. Errors: InvalidArgument (inconsistent profile data), ParseError (schema text), Unsupported
    // (a type this codec does not know how to write).
    static sdk::Result<PhotonCodec> create(const PhotonProfile& profile);

    const std::string& uuid() const;
    const std::string& version() const;
    // "photonstruct:PhotonPipelineResult:<uuid>", the type string of the rawBytes topic.
    std::string resultTypeString() const;

    // Encodes `value` as the message `message` (a photon message of the profile or a WPILib struct such as
    // "Transform3d", whose bytes are also the payload of the struct:Transform3d NetworkTables topics). `out` is
    // replaced; it is cleared when the value does not fit the schema (a missing member, a wrong kind, an
    // integer out of range for its width, a list above kPhotonMaxListLength).
    sdk::Status encode(std::string_view message, const PhotonValue& value, std::vector<std::uint8_t>& out) const;

    // The mirror of encode: refuses truncated input, lists above kPhotonMaxListLength and trailing bytes.
    // Floats are returned as reals, every integer width as an int.
    sdk::Result<PhotonValue> decode(std::string_view message, std::span<const std::uint8_t> bytes) const;

    struct Impl;  // the compiled schemas; opaque outside photon_codec.cpp

private:
    explicit PhotonCodec(std::shared_ptr<const Impl> impl) : impl_(std::move(impl)) {}

    std::shared_ptr<const Impl> impl_;
};

}  // namespace wpiemu::bridges::vision
