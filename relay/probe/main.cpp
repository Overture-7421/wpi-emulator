#include <cstring>
#include <iostream>

#include "wpiemu/relay/module.h"

int main(int argc, char** argv) {
    if (argc == 2 && std::strcmp(argv[1], "--help") == 0) {
        std::cout << "usage: wpiemu_relay_probe [--help]\n"
                  << "  (placeholder: the relay probe is not implemented yet; module " << wpiemu::relay::moduleName() << ")\n";
        return 0;
    }
    std::cerr << "wpiemu_relay_probe: not implemented yet; try --help\n";
    return 2;
}
