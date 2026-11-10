#pragma once

// Every sentence the client window says about a connection or a form, in one place and translated: the Qt-free layers hand
// over enums and stable keys, this turns them into text.

#include <QString>

#include "client/client_state.h"
#include "client/join_form.h"

namespace wpiemu::app::client {

QString bannerTitle(Banner banner);
// The line under the title: the host's reason for a refusal in plain words, the reconnect attempt, or the detail as sent.
QString bannerDetail(const Presentation& presentation);
QString rejectReasonText(sdk::match::JoinRejectReason reason);
QString fieldErrorText(const JoinFieldError& error);
QString stateWord(core::match::ConnectionState state);

}  // namespace wpiemu::app::client
