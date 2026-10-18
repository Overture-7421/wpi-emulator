#include "wpiemu/bridges/vision/nt_binding.h"

#include "wpiemu/bridges/vision/null_nt_api.h"

namespace wpiemu::bridges::vision {

// PLACEHOLDER owned by the nt-binding work stream, which replaces this whole file with the real binding.
std::unique_ptr<INtApi> makeNtCoreApi() { return std::make_unique<NullNtApi>("the ntcore binding is not built yet"); }

}  // namespace wpiemu::bridges::vision
