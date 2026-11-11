// wpiemu_relay_probe: checks a running relay from the outside. See relay/include/wpiemu/relay/relay_probe.h.

#include <iostream>
#include <string>
#include <vector>

#include "wpiemu/relay/relay_options.h"
#include "wpiemu/relay/relay_probe.h"

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv + 1, argv + argc);
    return wpiemu::relay::runProbeCli(args, wpiemu::relay::processEnvironment, std::cout, std::cerr, true);
}