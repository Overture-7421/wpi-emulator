#pragma once

#include <string>

namespace wpiemu::core::launcher {

// The Gradle init script of docs/reference/extension-ipc.md, passed with `-I` on the command line only (never installed into ~/.gradle/init.d):
//  * afterEvaluate: every wpi.sim HAL extension gets defaultEnabled=false (otherwise GradleRIO overwrites HALSIM_EXTENSIONS),
//  * simulation tasks, including 2027's JavaExec `run`, get HALSIM_EXTENSIONS = $WPIEMU_EXT.
//  * 2027's simulation environment map keeps that override, and its external Java descriptor gains the runtime classpath.
// The script holds no absolute paths: the library path and the IPC endpoint travel in the environment of the Gradle process
// (WPIEMU_EXT, WPIEMU_IPC_ENDPOINT, ...), so the file is identical for every run.
std::string generateInitScript();

}  // namespace wpiemu::core::launcher
