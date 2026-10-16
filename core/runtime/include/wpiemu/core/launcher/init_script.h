#pragma once

#include <string>

namespace wpiemu::core::launcher {

// The Gradle init script of ADR 0001, passed with `-I` on the command line only (never installed into ~/.gradle/init.d):
//  * afterEvaluate: every wpi.sim HAL extension gets defaultEnabled=false (otherwise GradleRIO overwrites HALSIM_EXTENSIONS),
//  * every simulateJava*/simulateNative* task gets HALSIM_EXTENSIONS = $WPIEMU_EXT [+ separator + $WPIEMU_EXTRA_EXT].
// The script holds no absolute paths: the library path and the IPC endpoint travel in the environment of the Gradle process
// (WPIEMU_EXT, WPIEMU_EXTRA_EXT, WPIEMU_IPC_ENDPOINT, ...), so the file is identical for every run.
std::string generateInitScript();

}  // namespace wpiemu::core::launcher
