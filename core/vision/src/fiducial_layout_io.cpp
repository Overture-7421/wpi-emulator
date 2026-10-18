#include "wpiemu/core/vision/fiducial_layout_io.h"

#include <cmath>
#include <exception>
#include <fstream>
#include <limits>
#include <set>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace wpiemu::core::vision {

using namespace wpiemu::sdk;

namespace {

namespace fs = std::filesystem;

constexpr std::uintmax_t kMaxLayoutFileBytes = std::uintmax_t{16} << 20;

Error bad(const std::string& path, const std::string& what) {
    return makeError(ErrorCode::ParseError, path + ": " + what);
}

Result<const Value*> objectMember(const Value& parent, std::string_view key,
                                  const std::string& parentPath) {
    const std::string path = parentPath + "." + std::string(key);
    const Value* v = parent.find(key);
    if (v == nullptr) return bad(path, "missing");
    if (!v->isObject()) return bad(path, "expected an object");
    return v;
}

Result<double> numberMember(const Value& parent, std::string_view key,
                            const std::string& parentPath) {
    const std::string path = parentPath + "." + std::string(key);
    const Value* v = parent.find(key);
    if (v == nullptr) return bad(path, "missing");
    if (!v->isNumber()) return bad(path, "expected a number");
    const double d = v->asDouble();
    if (!std::isfinite(d)) return bad(path, "must be a finite number");
    return d;
}

Result<FiducialTag> readTag(const Value& entry, const std::string& path) {
    if (!entry.isObject()) return bad(path, "expected an object");

    const Value* id = entry.find("ID");
    if (id == nullptr) return bad(path + ".ID", "missing");
    if (!id->isNumber()) return bad(path + ".ID", "expected an integer");
    const double idValue = id->asDouble();
    if (!(idValue >= 0.0 && idValue <= std::numeric_limits<int>::max()) ||
        idValue != std::floor(idValue))
        return bad(path + ".ID", "must be a non-negative integer");

    auto pose = objectMember(entry, "pose", path);
    if (!pose) return pose.error();
    const std::string posePath = path + ".pose";
    auto translation = objectMember(*pose.value(), "translation", posePath);
    if (!translation) return translation.error();
    auto rotation = objectMember(*pose.value(), "rotation", posePath);
    if (!rotation) return rotation.error();
    auto quaternion = objectMember(*rotation.value(), "quaternion", posePath + ".rotation");
    if (!quaternion) return quaternion.error();

    double t[3] = {};
    const char* const tKeys[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
        auto v = numberMember(*translation.value(), tKeys[i], posePath + ".translation");
        if (!v) return v.error();
        t[i] = v.value();
    }
    double q[4] = {};  // W, X, Y, Z as in the file
    const char* const qKeys[4] = {"W", "X", "Y", "Z"};
    for (int i = 0; i < 4; ++i) {
        auto v = numberMember(*quaternion.value(), qKeys[i], posePath + ".rotation.quaternion");
        if (!v) return v.error();
        q[i] = v.value();
    }
    const double length = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!(length > 1e-6))
        return bad(posePath + ".rotation.quaternion", "the quaternion must not be zero");

    FiducialTag tag;
    tag.id = static_cast<int>(idValue);
    tag.pose.position =
        Vec3{static_cast<float>(t[0]), static_cast<float>(t[1]), static_cast<float>(t[2])};
    tag.pose.rotation = Quat{static_cast<float>(q[1] / length), static_cast<float>(q[2] / length),
                             static_cast<float>(q[3] / length), static_cast<float>(q[0] / length)};
    return tag;
}

Result<FiducialLayout> layoutFromValue(const Value& root) {
    if (!root.isObject()) return bad("layout", "expected a JSON object");
    const Value* tags = root.find("tags");
    if (tags == nullptr) return bad("tags", "missing");
    if (!tags->isArray()) return bad("tags", "expected an array");
    if (tags->array().empty()) return bad("tags", "the layout has no tags");

    FiducialLayout layout;
    layout.id = "custom";
    layout.displayName = "Custom layout";
    std::set<int> seen;
    for (std::size_t i = 0; i < tags->array().size(); ++i) {
        auto tag = readTag(tags->array()[i], "tags[" + std::to_string(i) + "]");
        if (!tag) return tag.error();
        if (!seen.insert(tag.value().id).second)
            return bad("tags[" + std::to_string(i) + "].ID",
                       "duplicate tag id " + std::to_string(tag.value().id));
        layout.tags.push_back(tag.value());
    }

    if (const Value* field = root.find("field"); field != nullptr && !field->isNull()) {
        if (!field->isObject()) return bad("field", "expected an object");
        const std::pair<const char*, float*> dimensions[] = {{"length", &layout.fieldLengthM},
                                                              {"width", &layout.fieldWidthM}};
        for (const auto& [key, destination] : dimensions) {
            const Value* v = field->find(key);
            if (v == nullptr) continue;
            if (!v->isNumber() || !std::isfinite(v->asDouble()) || v->asDouble() < 0.0)
                return bad(std::string("field.") + key, "expected a finite, non-negative number");
            *destination = static_cast<float>(v->asDouble());
        }
    }
    return layout;
}

}  // namespace

Result<FiducialLayout> parseFiducialLayout(std::string_view jsonText,
                                           const IValueTextCodec& codec) {
    auto parsed = codec.parse(jsonText);
    if (!parsed) return parsed.error();
    return layoutFromValue(parsed.value());
}

Result<FiducialLayout> loadFiducialLayoutFile(const fs::path& path, const IValueTextCodec& codec) {
    try {
        const std::string name = path.string();
        const std::string what = "AprilTag layout file " + name;
        std::error_code ec;
        if (!fs::exists(path, ec)) return makeError(ErrorCode::NotFound, what + " not found");
        if (fs::is_directory(path, ec))
            return makeError(ErrorCode::IoError, "AprilTag layout path is a directory: " + name);
        const std::uintmax_t size = fs::file_size(path, ec);
        if (ec) return makeError(ErrorCode::IoError, "cannot read " + what + ": " + ec.message());
        if (size > kMaxLayoutFileBytes)
            return makeError(ErrorCode::IoError, what + " is too large");

        std::ifstream in(path, std::ios::binary);
        if (!in) return makeError(ErrorCode::IoError, "cannot open " + what);
        std::string text(static_cast<std::size_t>(size), '\0');
        in.read(text.data(), static_cast<std::streamsize>(text.size()));
        if (static_cast<std::uintmax_t>(in.gcount()) != size)
            return makeError(ErrorCode::IoError, "cannot read " + what);

        auto layout = parseFiducialLayout(text, codec);
        if (!layout) return makeError(layout.error().code, what + ": " + layout.error().message);
        layout.value().id = "file:" + path.filename().string();
        layout.value().displayName = path.stem().string();
        return layout;
    } catch (const std::exception& e) {
        return makeError(ErrorCode::IoError, std::string("AprilTag layout file: ") + e.what());
    }
}

Result<FiducialLayout> resolveFiducialLayout(const VisionConfig& config, const IGameModule* module,
                                             const IValueTextCodec& codec,
                                             const fs::path& baseDir) {
    if (!config.layoutFile.empty()) {
        fs::path path(config.layoutFile);
        if (path.is_relative() && !baseDir.empty()) path = baseDir / path;
        return loadFiducialLayoutFile(path, codec);
    }
    if (module == nullptr)
        return makeError(ErrorCode::NotFound,
                         "no AprilTag layout: select a game plugin that provides one or set "
                         "vision.layoutFile");

    const std::string plugin = "game plugin '" + module->metadata().id + "'";
    std::vector<FiducialLayout> layouts = module->contributeFiducialLayouts();
    if (!config.layoutId.empty()) {
        for (FiducialLayout& layout : layouts)
            if (layout.id == config.layoutId) return std::move(layout);
        std::string known;
        for (const FiducialLayout& layout : layouts)
            known += (known.empty() ? "" : ", ") + layout.id;
        return makeError(ErrorCode::NotFound,
                         plugin + " has no AprilTag layout '" + config.layoutId + "'" +
                             (known.empty() ? "" : " (available: " + known + ")"));
    }
    if (layouts.empty())
        return makeError(ErrorCode::NotFound,
                         plugin + " provides no AprilTag layout: set vision.layoutFile");
    return std::move(layouts.front());
}

}  // namespace wpiemu::core::vision
