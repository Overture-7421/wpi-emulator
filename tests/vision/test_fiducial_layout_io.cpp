#include <doctest/doctest.h>

#include <filesystem>
#include <fstream>
#include <random>
#include <string>

#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/core/vision/fiducial_layout_io.h"
#include "wpiemu/sdk/testing/fake_game_module.h"

using doctest::Approx;
namespace sdk = wpiemu::sdk;
namespace vis = wpiemu::core::vision;
using wpiemu::core::config::JsonValueCodec;

namespace {

// A scratch directory under the working directory (never the system temp dir), removed on exit.
class ScratchDir {
public:
    ScratchDir() {
        static std::mt19937_64 gen{std::random_device{}()};
        path_ = std::filesystem::current_path() / ("wpiemu-vision-scratch-" + std::to_string(gen() & 0xFFFFFFull));
        std::filesystem::create_directories(path_);
    }
    ~ScratchDir() {
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
    }
    ScratchDir(const ScratchDir&) = delete;
    ScratchDir& operator=(const ScratchDir&) = delete;
    const std::filesystem::path& path() const { return path_; }
    std::filesystem::path write(const std::string& name, const std::string& content) const {
        const std::filesystem::path p = path_ / name;
        std::ofstream(p, std::ios::binary) << content;
        return p;
    }

private:
    std::filesystem::path path_;
};

std::string tagJson(int id, double x, double y, double z, double w, double qx, double qy, double qz) {
    return "{\"ID\":" + std::to_string(id) + ",\"pose\":{\"translation\":{\"x\":" + std::to_string(x) +
           ",\"y\":" + std::to_string(y) + ",\"z\":" + std::to_string(z) +
           "},\"rotation\":{\"quaternion\":{\"W\":" + std::to_string(w) + ",\"X\":" + std::to_string(qx) +
           ",\"Y\":" + std::to_string(qy) + ",\"Z\":" + std::to_string(qz) + "}}}}";
}

const char* kTwoTags =
    R"({"tags":[
      {"ID":1,"pose":{"translation":{"x":1.0,"y":2.0,"z":0.5},
        "rotation":{"quaternion":{"W":0.7071067811865476,"X":0.0,"Y":0.0,"Z":0.7071067811865476}}}},
      {"ID":7,"pose":{"translation":{"x":16.0,"y":4.0,"z":1.2},
        "rotation":{"quaternion":{"W":1.0,"X":0.0,"Y":0.0,"Z":0.0}}}}],
      "field":{"length":16.541,"width":8.069}})";

sdk::Result<sdk::FiducialLayout> parse(const std::string& text) {
    return vis::parseFiducialLayout(text, JsonValueCodec{});
}

std::string errorOf(const std::string& text) {
    const auto r = parse(text);
    return r.ok() ? std::string() : r.error().message;
}

class LayoutModule final : public sdk::testing::FakeGameModule {
public:
    explicit LayoutModule(std::vector<sdk::FiducialLayout> layouts)
        : FakeGameModule("test_game", 2999), layouts_(std::move(layouts)) {}
    std::vector<sdk::FiducialLayout> contributeFiducialLayouts() const override { return layouts_; }

private:
    std::vector<sdk::FiducialLayout> layouts_;
};

sdk::FiducialLayout namedLayout(const std::string& id, int firstTagId) {
    sdk::FiducialLayout l;
    l.id = id;
    l.tags.push_back({firstTagId, {}, 0.0});
    return l;
}

}  // namespace

TEST_CASE("vision layout: a WPILib AprilTagFieldLayout file is read into tags, poses and field size") {
    const auto r = parse(kTwoTags);
    REQUIRE(r.ok());
    const sdk::FiducialLayout& l = r.value();
    CHECK(l.id == "custom");
    CHECK(l.tagSizeM == Approx(0.1651));  // the format carries no size: the default
    CHECK(l.fieldLengthM == Approx(16.541f));
    CHECK(l.fieldWidthM == Approx(8.069f));
    REQUIRE(l.tags.size() == 2);
    CHECK(l.tags[0].id == 1);  // file order
    CHECK(l.tags[1].id == 7);
    CHECK(l.tags[0].pose.position.x == Approx(1.f));
    CHECK(l.tags[0].pose.position.y == Approx(2.f));
    CHECK(l.tags[0].pose.position.z == Approx(0.5f));
    // W, X, Y, Z of the file become the SDK's (x, y, z, w): 90 degrees about +Z turns +X onto +Y
    const sdk::Vec3 normal = l.tags[0].pose.rotation.rotate({1.f, 0.f, 0.f});
    CHECK(normal.x == Approx(0.f).scale(1.0));
    CHECK(normal.y == Approx(1.f));
    CHECK(l.tags[1].pose.rotation == sdk::Quat{});
    CHECK(l.find(7) == &l.tags[1]);
    CHECK(l.find(2) == nullptr);
}

TEST_CASE("vision layout: quaternions are normalised and the field size is optional") {
    const std::string text = "{\"tags\":[" + tagJson(3, 0, 0, 0, 2.0, 0, 0, 0) + "," +
                             tagJson(4, 0, 0, 0, 1.0, 1.0, 1.0, 1.0) + "]}";
    const auto r = parse(text);
    REQUIRE(r.ok());
    const auto& q0 = r.value().tags[0].pose.rotation;
    CHECK(q0.w == Approx(1.f));
    CHECK(q0.x == 0.f);
    const auto& q1 = r.value().tags[1].pose.rotation;
    CHECK(q1.w == Approx(0.5f));
    CHECK(q1.x == Approx(0.5f));
    CHECK(q1.y == Approx(0.5f));
    CHECK(q1.z == Approx(0.5f));
    CHECK(r.value().fieldLengthM == 0.f);
    CHECK(r.value().fieldWidthM == 0.f);
}

TEST_CASE("vision layout: invalid content is an error that names the problem") {
    CHECK_FALSE(errorOf("").empty());
    CHECK_FALSE(errorOf("{not json").empty());
    CHECK(errorOf("[]").find("object") != std::string::npos);
    CHECK(errorOf("{}").find("tags") != std::string::npos);
    CHECK(errorOf("{\"tags\":{}}").find("array") != std::string::npos);
    CHECK(errorOf("{\"tags\":[]}").find("no tags") != std::string::npos);
    CHECK(errorOf("{\"tags\":[5]}").find("tags[0]") != std::string::npos);

    const std::string ok = tagJson(1, 1, 2, 3, 1, 0, 0, 0);
    CHECK(errorOf("{\"tags\":[" + ok + "]}").empty());
    CHECK(errorOf("{\"tags\":[" + ok + "," + tagJson(1, 4, 5, 6, 1, 0, 0, 0) + "]}").find("duplicate tag id 1") !=
          std::string::npos);

    // ID: present, integral and not negative
    CHECK(errorOf(R"({"tags":[{"pose":{}}]})").find("ID") != std::string::npos);
    CHECK(errorOf(R"({"tags":[{"ID":1.5,"pose":{}}]})").find("integer") != std::string::npos);
    CHECK(errorOf(R"({"tags":[{"ID":-2,"pose":{}}]})").find("non-negative") != std::string::npos);
    CHECK(errorOf(R"({"tags":[{"ID":"x","pose":{}}]})").find("ID") != std::string::npos);

    // pose members
    CHECK(errorOf(R"({"tags":[{"ID":1}]})").find("pose") != std::string::npos);
    CHECK(errorOf(R"({"tags":[{"ID":1,"pose":{"rotation":{"quaternion":{"W":1,"X":0,"Y":0,"Z":0}}}}]})")
              .find("translation") != std::string::npos);
    CHECK(errorOf(R"({"tags":[{"ID":1,"pose":{"translation":{"x":0,"y":0,"z":0}}}]})").find("rotation") !=
          std::string::npos);
    CHECK(errorOf(R"({"tags":[{"ID":1,"pose":{"translation":{"x":0,"y":0,"z":0},"rotation":{}}}]})")
              .find("quaternion") != std::string::npos);
    CHECK(errorOf(R"({"tags":[{"ID":1,"pose":{"translation":{"x":0,"y":0},"rotation":{"quaternion":{"W":1,"X":0,"Y":0,"Z":0}}}}]})")
              .find("translation.z") != std::string::npos);
    CHECK(errorOf(R"({"tags":[{"ID":1,"pose":{"translation":{"x":"a","y":0,"z":0},"rotation":{"quaternion":{"W":1,"X":0,"Y":0,"Z":0}}}}]})")
              .find("translation.x") != std::string::npos);
    CHECK(errorOf(R"({"tags":[{"ID":1,"pose":{"translation":{"x":0,"y":0,"z":0},"rotation":{"quaternion":{"W":1,"X":0,"Y":0}}}}]})")
              .find("quaternion.Z") != std::string::npos);

    // numbers that are not finite
    CHECK_FALSE(errorOf(R"({"tags":[{"ID":1,"pose":{"translation":{"x":1e999,"y":0,"z":0},"rotation":{"quaternion":{"W":1,"X":0,"Y":0,"Z":0}}}}]})").empty());

    // a zero quaternion cannot be normalised
    CHECK(errorOf("{\"tags\":[" + tagJson(1, 0, 0, 0, 0, 0, 0, 0) + "]}").find("quaternion") != std::string::npos);

    // the field size, when present, is a non-negative number
    CHECK(errorOf("{\"tags\":[" + ok + "],\"field\":5}").find("field") != std::string::npos);
    CHECK(errorOf("{\"tags\":[" + ok + R"(],"field":{"length":-1,"width":8}})").find("field.length") != std::string::npos);
    CHECK(errorOf("{\"tags\":[" + ok + R"(],"field":{"length":16,"width":"wide"}})").find("field.width") != std::string::npos);
    CHECK(errorOf("{\"tags\":[" + ok + R"(],"field":{"length":16.5,"width":8}})").empty());
}

TEST_CASE("vision layout: loading a file reports a missing file, a directory and bad content") {
    ScratchDir dir;
    const auto good = dir.write("custom_field.json", kTwoTags);
    const auto loaded = vis::loadFiducialLayoutFile(good, JsonValueCodec{});
    REQUIRE(loaded.ok());
    CHECK(loaded.value().id == "file:custom_field.json");
    CHECK(loaded.value().displayName == "custom_field");
    CHECK(loaded.value().tags.size() == 2);

    const auto missing = vis::loadFiducialLayoutFile(dir.path() / "nope.json", JsonValueCodec{});
    REQUIRE_FALSE(missing.ok());
    CHECK(missing.error().code == sdk::ErrorCode::NotFound);
    CHECK(missing.error().message.find("nope.json") != std::string::npos);

    const auto directory = vis::loadFiducialLayoutFile(dir.path(), JsonValueCodec{});
    REQUIRE_FALSE(directory.ok());
    CHECK(directory.error().code == sdk::ErrorCode::IoError);

    const auto badPath = dir.write("broken.json", "{\"tags\":[{\"ID\":1}]}");
    const auto broken = vis::loadFiducialLayoutFile(badPath, JsonValueCodec{});
    REQUIRE_FALSE(broken.ok());
    CHECK(broken.error().code == sdk::ErrorCode::ParseError);
    CHECK(broken.error().message.find("broken.json") != std::string::npos);
    CHECK(broken.error().message.find("tags[0].pose") != std::string::npos);

    const auto empty = dir.write("empty.json", "");
    CHECK_FALSE(vis::loadFiducialLayoutFile(empty, JsonValueCodec{}).ok());
}

TEST_CASE("vision layout: the project layout is the file, else the named plugin layout, else the plugin default") {
    ScratchDir dir;
    const auto file = dir.write("field.json", kTwoTags);
    LayoutModule module({namedLayout("test_game.welded", 100), namedLayout("test_game.andymark", 200)});
    const JsonValueCodec codec;

    sdk::VisionConfig config;
    // nothing set: the plugin's first layout
    auto r = vis::resolveFiducialLayout(config, &module, codec);
    REQUIRE(r.ok());
    CHECK(r.value().id == "test_game.welded");

    // a named layout
    config.layoutId = "test_game.andymark";
    r = vis::resolveFiducialLayout(config, &module, codec);
    REQUIRE(r.ok());
    CHECK(r.value().id == "test_game.andymark");
    CHECK(r.value().tags[0].id == 200);

    // an unknown name is an error that lists what the plugin has
    config.layoutId = "test_game.missing";
    r = vis::resolveFiducialLayout(config, &module, codec);
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == sdk::ErrorCode::NotFound);
    CHECK(r.error().message.find("test_game.missing") != std::string::npos);
    CHECK(r.error().message.find("test_game.welded") != std::string::npos);
    CHECK(r.error().message.find("test_game.andymark") != std::string::npos);

    // a file beats the name, with or without a plugin
    config.layoutFile = file.string();
    r = vis::resolveFiducialLayout(config, &module, codec);
    REQUIRE(r.ok());
    CHECK(r.value().id == "file:field.json");
    r = vis::resolveFiducialLayout(config, nullptr, codec);
    REQUIRE(r.ok());
    CHECK(r.value().tags.size() == 2);

    // a relative file is looked up below the base directory
    config.layoutFile = "field.json";
    r = vis::resolveFiducialLayout(config, &module, codec, dir.path());
    REQUIRE(r.ok());
    CHECK(r.value().id == "file:field.json");

    // a file that cannot be read is an error, not a silent fallback to the plugin
    config.layoutFile = (dir.path() / "gone.json").string();
    r = vis::resolveFiducialLayout(config, &module, codec);
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == sdk::ErrorCode::NotFound);
}

TEST_CASE("vision layout: without a file and without a usable plugin there is no layout") {
    const JsonValueCodec codec;
    sdk::VisionConfig config;
    auto r = vis::resolveFiducialLayout(config, nullptr, codec);
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().message.find("layoutFile") != std::string::npos);

    config.layoutId = "test_game.welded";  // a name needs a plugin
    CHECK_FALSE(vis::resolveFiducialLayout(config, nullptr, codec).ok());

    LayoutModule none({});  // a plugin without layouts
    config.layoutId.clear();
    r = vis::resolveFiducialLayout(config, &none, codec);
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().message.find("test_game") != std::string::npos);
    config.layoutId = "test_game.welded";
    CHECK_FALSE(vis::resolveFiducialLayout(config, &none, codec).ok());
}
