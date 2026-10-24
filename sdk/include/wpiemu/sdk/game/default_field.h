#pragma once

// The field of a season without a game plugin ("core-only"): carpet, two alliance walls and two
// guardrails around a generic FRC-sized playing area, so the robot has a floor and boundaries but
// no game structure. It names no season (docs/reference/physics.md#default-field). Blue alliance
// wall at x = 0, red at x = length; guardrails at y = 0 and y = width.

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/game/field.h"
#include "wpiemu/sdk/physics/materials.h"
#include "wpiemu/sdk/physics/physics_world.h"

namespace wpiemu::sdk {

namespace default_field {

inline constexpr float kLengthM = 16.46f;     // 54 ft, along +X
inline constexpr float kWidthM = 8.23f;       // 27 ft, along +Y
inline constexpr float kCarpetMarginM = 3.f;  // carpet extends past the walls on every side
inline constexpr float kWallThicknessM = 0.30f;
inline constexpr float kWallHeightM = 2.0f;
inline constexpr float kWallBaseHeightM = 0.93f;  // opaque alliance-coloured lower panel
inline constexpr float kGuardrailThicknessM = 0.05f;
inline constexpr float kGuardrailHeightM = 0.508f;

}  // namespace default_field

namespace default_field_detail {

struct Block {
    std::string name;
    Vec3 center;
    Vec3 size;  // full extents
    Material material;
};

inline constexpr const char* kCarpetColor = "#5a5f6b";
inline constexpr const char* kBlueColor = "#1f4fb3";
inline constexpr const char* kRedColor = "#b3261e";
inline constexpr const char* kPlasticColor = "#a9cde6";
inline constexpr const char* kAluminumColor = "#b9bec7";

// Every collider of the field. The walls span the guardrail thickness on both sides so the corners
// are sealed.
inline std::vector<Block> colliders() {
    using namespace default_field;
    const float len = kLengthM, wid = kWidthM, t = kWallThicknessM, g = kGuardrailThicknessM;
    const float wallSpan = wid + 2.f * g;
    std::vector<Block> out;
    out.push_back({"carpet",
                   {len / 2, wid / 2, -0.25f},
                   {len + 2 * kCarpetMarginM, wid + 2 * kCarpetMarginM, 0.5f},
                   materials::kCarpet});
    out.push_back({"wall_blue",
                   {-t / 2, wid / 2, kWallHeightM / 2},
                   {t, wallSpan, kWallHeightM},
                   materials::kFieldStructure});
    out.push_back({"wall_red",
                   {len + t / 2, wid / 2, kWallHeightM / 2},
                   {t, wallSpan, kWallHeightM},
                   materials::kFieldStructure});
    out.push_back({"guardrail_low_y",
                   {len / 2, -g / 2, kGuardrailHeightM / 2},
                   {len, g, kGuardrailHeightM},
                   materials::kFieldStructure});
    out.push_back({"guardrail_high_y",
                   {len / 2, wid + g / 2, kGuardrailHeightM / 2},
                   {len, g, kGuardrailHeightM},
                   materials::kFieldStructure});
    return out;
}

inline FieldVisual visual(std::string name, const Vec3& center, const Vec3& size, const char* color,
                          float opacity = 1.f) {
    FieldVisual v;
    v.name = std::move(name);
    v.pose = Pose3{center, Quat::identity()};
    v.sizeM = size;
    v.color = color;
    v.opacity = opacity;
    return v;
}

}  // namespace default_field_detail

// What the renderer draws: carpet, alliance walls (coloured base, see-through upper panel, top
// rail) and guardrails (see-through panel between aluminum rails).
inline FieldDescription defaultFieldDescription() {
    using namespace default_field;
    using namespace default_field_detail;
    const float len = kLengthM, wid = kWidthM, t = kWallThicknessM, g = kGuardrailThicknessM,
                h = kWallHeightM;
    const float base = kWallBaseHeightM, wallSpan = wid + 2.f * g, rail = 0.04f;

    FieldDescription d;
    d.name = "Generic FRC field";
    d.lengthM = len;
    d.widthM = wid;
    d.visuals.push_back(visual("carpet", {len / 2, wid / 2, -0.25f},
                               {len + 2 * kCarpetMarginM, wid + 2 * kCarpetMarginM, 0.5f},
                               kCarpetColor));
    for (const bool red : {false, true}) {
        const std::string n = red ? "wall_red" : "wall_blue";
        const float inner = red ? len : 0.f,
                    out = red ? 1.f : -1.f;  // inner face x and the direction away from the field
        d.visuals.push_back(visual(n + "_base", {inner + out * t / 2, wid / 2, base / 2},
                                   {t, wallSpan, base}, red ? kRedColor : kBlueColor));
        d.visuals.push_back(visual(n + "_sheet",
                                   {inner + out * 0.01f, wid / 2, (base + h - rail) / 2},
                                   {0.02f, wallSpan, h - rail - base}, kPlasticColor, 0.3f));
        d.visuals.push_back(visual(n + "_rail", {inner + out * t / 2, wid / 2, h - rail / 2},
                                   {t, wallSpan, rail}, kAluminumColor));
    }
    for (const bool high : {false, true}) {
        const std::string n = high ? "guardrail_high_y" : "guardrail_low_y";
        const float y = high ? wid + g / 2 : -g / 2;
        d.visuals.push_back(visual(n + "_panel", {len / 2, y, kGuardrailHeightM / 2},
                                   {len, 0.01f, kGuardrailHeightM - 2 * rail}, kPlasticColor,
                                   0.3f));
        d.visuals.push_back(
            visual(n + "_bottom", {len / 2, y, rail / 2}, {len, g, rail}, kAluminumColor));
        d.visuals.push_back(visual(n + "_top", {len / 2, y, kGuardrailHeightM - rail / 2},
                                   {len, g, rail}, kAluminumColor));
    }
    return d;
}

// Owns the static colliders created in a world; destroying it removes them, so it must be destroyed
// BEFORE the world.
class DefaultField final : public IField {
public:
    static Result<std::unique_ptr<IField>> create(IPhysicsWorld& world) {
        BodyDesc desc;
        desc.type = BodyType::Static;
        const BodyId body = world.createBody(desc);
        if (!body.valid())
            return makeError(ErrorCode::Internal, "physics world refused the default field body");
        std::unique_ptr<IField> field(new DefaultField(world, body));  // owns the body from here on
        for (const default_field_detail::Block& block : default_field_detail::colliders()) {
            ShapeDesc shape;
            shape.geometry = BoxShape{block.size * 0.5f};
            shape.local = Pose3{block.center, Quat::identity()};
            shape.material = block.material;
            shape.category = layer::Field;
            if (!world.addShape(body, shape).valid())
                return makeError(ErrorCode::Internal,
                                 "physics world refused default field shape: " + block.name);
        }
        return field;
    }

    ~DefaultField() override {
        if (body_.valid()) world_.destroyBody(body_);
    }
    DefaultField(const DefaultField&) = delete;
    DefaultField& operator=(const DefaultField&) = delete;

    const FieldDescription& description() const override { return description_; }

private:
    DefaultField(IPhysicsWorld& world, BodyId body)
        : world_(world), body_(body), description_(defaultFieldDescription()) {}

    IPhysicsWorld& world_;
    BodyId body_;
    FieldDescription description_;
};

inline Result<std::unique_ptr<IField>> createDefaultField(IPhysicsWorld& world) {
    return DefaultField::create(world);
}

}  // namespace wpiemu::sdk
