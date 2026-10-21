#include <doctest/doctest.h>

#include <string>

#include "wpiemu/sdk/config/plugin_config.h"
#include "wpiemu/sdk/testing/demo_game.h"

using namespace wpiemu::sdk;
namespace demo = wpiemu::sdk::testing::demo_game;

TEST_CASE("demo game: every id it declares is owned by the demo plugin id") {
    const auto owned = [](const std::string& id) { return ownsPluginId(demo::kPluginId, id); };
    for (const ZoneKindInfo& kind : demo::zoneKinds()) CHECK(owned(kind.kind));
    for (const GameValueInfo& value : demo::gameValues()) CHECK(owned(value.id));
    for (const EditorPanelDescriptor& panel : demo::editorPanels("sample.shooter")) CHECK(owned(panel.id));
    for (const HudWidgetDescriptor& widget : demo::hud()) {
        CHECK(owned(widget.id));
        CHECK(owned(widget.valueKey));
        if (!widget.maxKey.empty()) CHECK(owned(widget.maxKey));
        for (const HudScoreSide& side : widget.sides) {
            CHECK(owned(side.scoreKey));
            CHECK(owned(side.statusKey));
        }
    }
}

TEST_CASE("demo game: the field, its frame and its tags agree") {
    const FieldDescription field = demo::field();
    const FieldFrame frame = demo::fieldFrame();
    const AprilTagLayout tags = demo::aprilTags();
    CHECK(field.lengthM == frame.lengthM);
    CHECK(field.widthM == frame.widthM);
    CHECK(tags.fieldLengthM == frame.lengthM);
    CHECK(tags.fieldWidthM == frame.widthM);
    CHECK(frame.symmetry != FieldSymmetry::None);

    REQUIRE(tags.tags.size() == 8);
    for (const AprilTag& tag : tags.tags) {
        CHECK(tag.sizeM == kFrcAprilTagSizeM);
        CHECK(tag.pose.position.x >= 0.f);
        CHECK(tag.pose.position.x <= frame.lengthM);
        CHECK(tag.pose.position.y >= 0.f);
        CHECK(tag.pose.position.y <= frame.widthM);
    }
}
