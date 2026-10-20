#include "contract/runtime_host.h"

namespace wpiemu::app {

void registerHostMetaTypes() {
    static const bool done = [] {
        qRegisterMetaType<HostLogLine>();
        qRegisterMetaType<sdk::proto::DsState>();
        return true;
    }();
    (void)done;
}

IRuntimeHost::IRuntimeHost(QObject* parent) : QObject(parent) { registerHostMetaTypes(); }

}  // namespace wpiemu::app
