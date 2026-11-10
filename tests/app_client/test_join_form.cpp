// The join form: invite format, direct address, the relay choice (Production default, Development, custom) and the rest.
#include <doctest/doctest.h>

#include <algorithm>
#include <cctype>
#include <string>

#include "client/join_form.h"

using namespace wpiemu;
using namespace wpiemu::app;
using namespace wpiemu::app::client;

namespace {

const net::InviteSecret& secret() {
    static const net::InviteSecret s = [] {
        net::InviteSecret::Bytes bytes{};
        for (std::size_t i = 0; i < bytes.size(); ++i) bytes[i] = static_cast<std::uint8_t>(0x11 * (i + 1));
        bytes.back() &= 0xF0;
        return net::InviteSecret::fromBytes(bytes).value();
    }();
    return s;
}

JoinFormInput goodForm() {
    JoinFormInput in;
    in.invite = net::inviteText(secret());
    in.displayName = "Iron Owls";
    in.robotPath = "C:/robots/robot.json";
    return in;
}

RelayPresets presets() { return relayPresetsFor("-ab12"); }

}  // namespace

TEST_CASE("join form: a complete form joins through the production relay by default") {
    const JoinValidation v = validateJoin(goodForm(), presets());
    REQUIRE(v.ok());
    CHECK(v.errors.empty());
    CHECK_FALSE(v.plan->direct());
    CHECK(v.plan->endpoint.scheme == net::EndpointScheme::Relay);
    CHECK(v.plan->endpoint.host == "wpiemu-relay-ab12.southcentralus.azurecontainer.io");
    CHECK(v.plan->endpoint.port == 7777);
    CHECK(v.plan->endpoint.roomId == net::roomId(secret()));   // the relay sees only the room derived from the invite
    CHECK(v.plan->secret == secret());
    CHECK(v.plan->displayName == "Iron Owls");
}

TEST_CASE("join form: the invite is forgiving about case, dashes, spaces and look-alike characters") {
    JoinFormInput in = goodForm();
    std::string messy = net::inviteText(secret());
    for (char& c : messy) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    for (const std::string& text : {messy, " " + messy + " ", net::inviteText(secret())}) {
        in.invite = text;
        const JoinValidation v = validateJoin(in, presets());
        REQUIRE(v.ok());
        CHECK(v.plan->secret == secret());
    }
    std::string bare = net::inviteText(secret());
    bare.erase(std::remove(bare.begin(), bare.end(), '-'), bare.end());
    in.invite = bare;
    CHECK(validateJoin(in, presets()).ok());
}

TEST_CASE("join form: a wrong invite format is reported on the invite field with a stable key") {
    JoinFormInput in = goodForm();
    for (const char* text : {"ABCDE-FGHJK", "not an invite at all", "ABCDE-FGHJK-MNPQR-STVWX-YZ012", "ABCDE-FGHJK-MNPQR-STVW!"}) {
        CAPTURE(text);
        in.invite = text;
        const JoinValidation v = validateJoin(in, presets());
        CHECK_FALSE(v.ok());
        const JoinFieldError* e = v.errorOf(JoinField::Invite);
        REQUIRE(e != nullptr);
        CHECK(e->key == "invite.format");
        CHECK_FALSE(e->detail.empty());
    }
    in.invite = "   ";
    const JoinValidation empty = validateJoin(in, presets());
    REQUIRE(empty.errorOf(JoinField::Invite) != nullptr);
    CHECK(empty.errorOf(JoinField::Invite)->key == "invite.empty");
}

TEST_CASE("join form: a direct address overrides the relay and defaults the port") {
    JoinFormInput in = goodForm();
    in.directAddress = "192.168.1.20";
    JoinValidation v = validateJoin(in, presets());
    REQUIRE(v.ok());
    CHECK(v.plan->direct());
    CHECK(v.plan->endpoint.format() == "enet://192.168.1.20:47300");
    CHECK(v.plan->endpoint.roomId.empty());

    in.directAddress = " robots.example.org:5000 ";
    v = validateJoin(in, presets());
    REQUIRE(v.ok());
    CHECK(v.plan->endpoint.host == "robots.example.org");
    CHECK(v.plan->endpoint.port == 5000);

    in.directAddress = "[::1]:6000";
    v = validateJoin(in, presets());
    REQUIRE(v.ok());
    CHECK(v.plan->endpoint.host == "::1");

    in.directAddress = "bad host:1";
    v = validateJoin(in, presets());
    CHECK_FALSE(v.ok());
    REQUIRE(v.errorOf(JoinField::DirectAddress) != nullptr);
    CHECK(v.errorOf(JoinField::DirectAddress)->key == "direct.address");
    in.directAddress = "host:70000";
    CHECK_FALSE(validateJoin(in, presets()).ok());
}

TEST_CASE("join form: the relay choice picks the preset or the custom address") {
    JoinFormInput in = goodForm();
    in.relay = RelayChoice::Development;
    JoinValidation v = validateJoin(in, presets());
    REQUIRE(v.ok());
    CHECK(v.plan->endpoint.host == "wpiemu-relay-dev-ab12.southcentralus.azurecontainer.io");

    in.relay = RelayChoice::Custom;
    in.customRelay = "relay.example.org:7000";
    v = validateJoin(in, presets());
    REQUIRE(v.ok());
    CHECK(v.plan->endpoint.format() == "relay://relay.example.org:7000/" + net::roomId(secret()));
    in.customRelay = "relay://relay.example.org:7000";
    CHECK(validateJoin(in, presets()).ok());

    for (const char* bad : {"", "relay.example.org", "relay.example.org:0", "enet://relay.example.org:7000"}) {
        CAPTURE(bad);
        in.customRelay = bad;
        v = validateJoin(in, presets());
        CHECK_FALSE(v.ok());
        REQUIRE(v.errorOf(JoinField::Relay) != nullptr);
        CHECK(v.errorOf(JoinField::Relay)->key == "relay.address");
    }
}

TEST_CASE("join form: without a DNS suffix the presets are unknown and the form says to use a custom relay") {
    JoinFormInput in = goodForm();
    const JoinValidation v = validateJoin(in, relayPresetsFor(""));
    CHECK_FALSE(v.ok());
    REQUIRE(v.errorOf(JoinField::Relay) != nullptr);
    CHECK(v.errorOf(JoinField::Relay)->key == "relay.unconfigured");
    in.directAddress = "10.0.0.5:47300";   // a direct connection needs no relay at all
    CHECK(validateJoin(in, relayPresetsFor("")).ok());
}

TEST_CASE("join form: the name and the robot file are required, and every problem is listed at once") {
    JoinFormInput in = goodForm();
    in.displayName = "   ";
    in.robotPath = "";
    in.invite = "";
    const JoinValidation v = validateJoin(in, presets());
    CHECK_FALSE(v.ok());
    CHECK(v.errors.size() == 3);
    CHECK(v.errorOf(JoinField::DisplayName)->key == "name.empty");
    CHECK(v.errorOf(JoinField::Robot)->key == "robot.missing");
    CHECK(v.errorOf(JoinField::Invite)->key == "invite.empty");

    in = goodForm();
    in.displayName = std::string(wpiemu::sdk::match::kMaxNameBytes + 1, 'x');
    CHECK(validateJoin(in, presets()).errorOf(JoinField::DisplayName)->key == "name.too_long");
    in.displayName = std::string(wpiemu::sdk::match::kMaxNameBytes, 'x');
    CHECK(validateJoin(in, presets()).ok());
}

TEST_CASE("join form: the relay address of a choice") {
    JoinFormInput in = goodForm();
    CHECK(relayAddressOf(in, presets()) == presets().production);
    in.relay = RelayChoice::Development;
    CHECK(relayAddressOf(in, presets()) == presets().development);
    in.relay = RelayChoice::Custom;
    in.customRelay = "  a.example:1  ";
    CHECK(relayAddressOf(in, presets()) == "a.example:1");
}
