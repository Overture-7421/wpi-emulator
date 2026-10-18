#include <doctest/doctest.h>

#include <string>
#include <vector>

#include "photon_test_support.h"
#include "wpiemu/bridges/vision/photon_codec.h"
#include "wpiemu/bridges/vision/photon_md5.h"
#include "wpiemu/bridges/vision/photon_profile.h"
#include "wpiemu/sdk/vision/photon_profiles.h"

namespace {

namespace sdk = wpiemu::sdk;
using namespace wpiemu::bridges::vision;
using wpiemu::sdk::ErrorCode;
using wpiemu::tests::photon::MessageSpec;
using wpiemu::tests::photon::OwnedProfile;
using wpiemu::tests::photon::realMessageSpecs;

TEST_CASE("photon md5: the RFC 1321 test vectors, including inputs that span two blocks") {
    CHECK(md5Hex("") == "d41d8cd98f00b204e9800998ecf8427e");
    CHECK(md5Hex("a") == "0cc175b9c0f1b6a831c399e269772661");
    CHECK(md5Hex("abc") == "900150983cd24fb0d6963f7d28e17f72");
    CHECK(md5Hex("message digest") == "f96b697d7cb7938d525a2f31aaf161d0");
    CHECK(md5Hex("abcdefghijklmnopqrstuvwxyz") == "c3fcd3d76192e4007dfb496cca67e13b");
    CHECK(md5Hex("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789") ==
          "d174ab98d277d9f5a5611c2c9f419d9f");
    CHECK(md5Hex("12345678901234567890123456789012345678901234567890123456789012345678901234567890") ==
          "57edf4a22be3c955ac49da2e2107b67a");
    // The padding boundaries: 55 / 56 / 63 / 64 bytes of 'a', and tails that need a second padding block.
    CHECK(md5Hex(std::string(55, 'a')) == "ef1772b6dff9a122358552954ad0df65");
    CHECK(md5Hex(std::string(56, 'a')) == "3b0c8ac703f828b04c6c197006d17218");
    CHECK(md5Hex(std::string(63, 'a')) == "b06521f39153d618550606be297466d5");
    CHECK(md5Hex(std::string(64, 'a')) == "014842d480b571495a4a0363793f7367");
    CHECK(md5Hex(std::string(119, 'a')) == "8a7bd0732ed6a28ce75f6dabc90e1613");
    CHECK(md5Hex(std::string(120, 'a')) == "5f61c0ccad4cac44c75ff505e1f1e537");
    CHECK(md5Hex(std::string(128, 'a')) == "e510683b3f5ffe4093d021808bc6ff70");
}

TEST_CASE("photon profile: every message of every known profile hashes to its UUID and the codec accepts it") {
    REQUIRE_FALSE(knownPhotonProfiles().empty());
    for (const PhotonProfile& profile : knownPhotonProfiles()) {
        CAPTURE(std::string(profile.uuid));
        CHECK_FALSE(profile.version.empty());
        for (const PhotonMessageDef& message : profile.messages) {
            CAPTURE(std::string(message.name));
            CHECK(md5Hex(message.schema) == message.uuid);
        }
        const auto codec = PhotonCodec::create(profile);
        REQUIRE_MESSAGE(codec.ok(), wpiemu::tests::photon::why(codec));
        CHECK(codec.value().uuid() == profile.uuid);
        CHECK(codec.value().version() == profile.version);
        CHECK(codec.value().resultTypeString() == "photonstruct:PhotonPipelineResult:" + std::string(profile.uuid));
    }
}

TEST_CASE("photon profile: the known profile is the one of PhotonLib v2025.3.2 to v2026.3.4") {
    const PhotonProfile* profile = findPhotonProfile("4b2ff16a964b5e2bf04be0c1454d91c4");
    REQUIRE(profile != nullptr);
    CHECK(profile == &newestPhotonProfile());
    CHECK(profile->version == "v2026.3.4");
    CHECK(profile->messages.size() == 6);
    CHECK(photonTypeString(kPhotonResultMessage, profile->uuid) ==
          "photonstruct:PhotonPipelineResult:4b2ff16a964b5e2bf04be0c1454d91c4");
}

TEST_CASE("photon profile: unknown and empty UUIDs have no profile, capitals are the same UUID") {
    CHECK(findPhotonProfile("") == nullptr);
    CHECK(findPhotonProfile("deadbeefdeadbeefdeadbeefdeadbeef") == nullptr);
    CHECK(findPhotonProfile("unverified") == nullptr);
    CHECK(findPhotonProfile("4b2ff16a964b5e2bf04be0c1454d91c") == nullptr);  // one digit short
    // Hex text of a hand-edited robot.json: the profile is found, and it keeps its canonical lower case spelling.
    const PhotonProfile* capitals = findPhotonProfile("4B2FF16A964B5E2BF04BE0C1454D91C4");
    REQUIRE(capitals != nullptr);
    CHECK(capitals == findPhotonProfile("4b2ff16a964b5e2bf04be0c1454d91c4"));
    CHECK(capitals->uuid == "4b2ff16a964b5e2bf04be0c1454d91c4");
}

TEST_CASE("photon profile: every UUID the SDK's version rule returns has a profile") {
    for (const char* version :
         {"v2025.3.1", "2025.3.2", "v2025.9.9", "v2026.1.1", "v2026.2.9", "v2026.3.4", "2026.3.0"}) {
        CAPTURE(version);
        const std::string uuid = wpiemu::sdk::photonMessageUuidForVersion(version);
        REQUIRE_FALSE(uuid.empty());
        CHECK(findPhotonProfile(uuid) != nullptr);
    }
    CHECK(wpiemu::sdk::photonMessageUuidForVersion("v2026.3.5").empty());  // newer than verified: no profile is claimed
}

TEST_CASE("photon profile: the texts equal the message definitions the real PhotonLib reports") {
    for (const PhotonProfile& profile : knownPhotonProfiles()) {
        CAPTURE(std::string(profile.uuid));
        const auto path = wpiemu::tests::photon::fixtureDirectory(profile.uuid) / "schemas.txt";
        REQUIRE_MESSAGE(
            std::filesystem::is_regular_file(path),
            "no real-library fixtures for the profile: run tools/gen_photon_fixtures.py (" << path.string() << ")");
        std::size_t seen = 0;
        for (const std::string& line : wpiemu::tests::photon::readFixtureLines(path)) {
            const auto fields = wpiemu::tests::photon::splitWords(line, '\t');
            if (fields[0] == "version") {
                REQUIRE(fields.size() == 2);
                CHECK(fields[1] == profile.version);
                continue;
            }
            if (fields[0] == "maxlist") {
                CHECK(std::stoul(fields[1]) == kPhotonMaxListLength);
                continue;
            }
            REQUIRE(fields.size() == 3);
            const auto def =
                std::find_if(profile.messages.begin(), profile.messages.end(),
                             [&](const PhotonMessageDef& candidate) { return candidate.name == fields[0]; });
            REQUIRE_MESSAGE(def != profile.messages.end(),
                            "the real library has a message the profile lacks: " << fields[0]);
            CHECK(def->uuid == fields[1]);
            CHECK(def->schema == fields[2]);
            ++seen;
        }
        CHECK(seen == profile.messages.size());  // and the profile has no message the library does not have
    }
}

// ---- profiles the codec refuses ------------------------------------------------------------------------------

std::vector<MessageSpec> realSpecsWith(const std::string& name, const std::string& schema) {
    std::vector<MessageSpec> specs = realMessageSpecs();
    for (MessageSpec& spec : specs) {
        if (spec.name == name) spec.schema = schema;
    }
    return specs;
}

// The error of PhotonCodec::create for specs that must be refused.
sdk::Error refusal(const std::vector<MessageSpec>& specs) {
    const OwnedProfile owned(specs);
    const auto codec = PhotonCodec::create(owned.profile());
    REQUIRE_FALSE(codec.ok());
    return codec.error();
}

TEST_CASE("photon codec: a profile rebuilt from the real texts gets the real UUIDs back") {
    const OwnedProfile owned(realMessageSpecs());
    CHECK(owned.uuid() == newestPhotonProfile().uuid);
    const auto codec = PhotonCodec::create(owned.profile());
    CHECK(codec.ok());
}

TEST_CASE("photon codec: a text that does not hash to its UUID is refused") {
    std::vector<PhotonMessageDef> messages(newestPhotonProfile().messages.begin(),
                                           newestPhotonProfile().messages.end());
    for (PhotonMessageDef& message : messages) {
        if (message.name == "PhotonPipelineMetadata") {
            message.schema =
                "int64 sequenceID;int64 captureTimestampMicros;int64 publishTimestampMicros;int64 "
                "timeSinceLastPong;int64 extra;";
        }
    }
    const auto codec = PhotonCodec::create(PhotonProfile{newestPhotonProfile().uuid, "v-test", messages});
    REQUIRE_FALSE(codec.ok());
    CHECK(codec.error().code == ErrorCode::InvalidArgument);
    CHECK(codec.error().message.find("PhotonPipelineMetadata") != std::string::npos);
    CHECK(codec.error().message.find("MD5") != std::string::npos);
}

TEST_CASE("photon codec: inconsistent profile data is refused") {
    const PhotonProfile& real = newestPhotonProfile();
    CHECK(PhotonCodec::create(PhotonProfile{real.uuid, "v-test", {}}).error().code == ErrorCode::InvalidArgument);
    CHECK(PhotonCodec::create(PhotonProfile{real.uuid, "", real.messages}).error().code ==
          ErrorCode::InvalidArgument);  // no version text
    CHECK(
        PhotonCodec::create(PhotonProfile{"deadbeefdeadbeefdeadbeefdeadbeef", "v-test", real.messages}).error().code ==
        ErrorCode::InvalidArgument);  // the root's UUID is not the profile's

    SUBCASE("the result message is missing") {
        std::vector<MessageSpec> specs = realMessageSpecs();
        specs.pop_back();  // PhotonPipelineResult comes last
        CHECK(refusal(specs).code == ErrorCode::InvalidArgument);
    }
    SUBCASE("a message is defined twice") {
        std::vector<MessageSpec> specs = realMessageSpecs();
        specs.insert(specs.begin(), specs.front());
        CHECK(refusal(specs).code == ErrorCode::InvalidArgument);
    }
    SUBCASE("a message takes the name of a WPILib struct") {
        std::vector<MessageSpec> specs = realMessageSpecs();
        specs.insert(specs.begin(), MessageSpec{"Transform3d", "float64 x;"});
        CHECK(refusal(specs).code == ErrorCode::InvalidArgument);
    }
    SUBCASE("a message name that is not an identifier") {
        std::vector<MessageSpec> specs = realMessageSpecs();
        specs.insert(specs.begin(), MessageSpec{"Not A Name", "float64 x;"});
        CHECK(refusal(specs).code == ErrorCode::InvalidArgument);
    }
    SUBCASE("a UUID that is not 32 lower-case hex digits") {
        const PhotonMessageDef bad[] = {{"PhotonPipelineResult", "4B2FF16A964B5E2BF04BE0C1454D91C4", "float64 x;"}};
        CHECK(PhotonCodec::create(PhotonProfile{"4B2FF16A964B5E2BF04BE0C1454D91C4", "v-test", bad}).error().code ==
              ErrorCode::InvalidArgument);
    }
}

TEST_CASE("photon codec: malformed schema texts are parse errors") {
    const std::vector<std::string> texts = {
        "float64 x;float64 y",      // a field must end with ';'
        "float64 x;;float64 y;",    // an empty field
        "float64 x y;",             // not '<type> <name>'
        "float64;",                 // no name
        "float64 x[3];float64 y;",  // fixed length arrays do not exist in photon schemas
        "optional float64 x[?];",   // a field is optional or a list, not both
        "float64 9x;float64 y;",    // not an identifier
        "float64 x;Foo:xyz y;",     // a UUID after ':' is 32 lower-case hex digits
        "flo-at64 x;float64 y;",    // not an identifier
    };
    for (const std::string& text : texts) {
        CAPTURE(text);
        const sdk::Error error = refusal(realSpecsWith("TargetCorner", text));
        CHECK(error.code == ErrorCode::ParseError);
        CHECK(error.message.find("TargetCorner") != std::string::npos);
    }
}

TEST_CASE("photon codec: types the codec cannot write are unsupported, references that do not match are invalid") {
    CHECK(refusal(realSpecsWith("TargetCorner", "Pose2d x;float64 y;")).code == ErrorCode::Unsupported);
    CHECK(refusal(realSpecsWith("TargetCorner", "int8 x;float64 y;")).code == ErrorCode::Unsupported);
    CHECK(refusal(realSpecsWith("TargetCorner", "float64:0123456789abcdef0123456789abcdef x;float64 y;")).code ==
          ErrorCode::InvalidArgument);
    // PhotonTrackedTarget names a TargetCorner UUID the profile does not carry.
    std::vector<MessageSpec> specs = realMessageSpecs();
    for (MessageSpec& spec : specs) {
        if (spec.name == "PhotonTrackedTarget") {
            spec.schema = "float64 yaw;TargetCorner:0123456789abcdef0123456789abcdef corners[?];";
        }
    }
    const sdk::Error mismatch = refusal(specs);
    CHECK(mismatch.code == ErrorCode::InvalidArgument);
    CHECK(mismatch.message.find("TargetCorner") != std::string::npos);
    // ... or a message the profile does not define at all.
    specs.erase(
        std::remove_if(specs.begin(), specs.end(), [](const MessageSpec& spec) { return spec.name == "TargetCorner"; }),
        specs.end());
    CHECK(refusal(specs).code == ErrorCode::InvalidArgument);
}

}  // namespace
