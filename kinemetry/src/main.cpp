#include <iostream>

#include "kinecore/version.hpp"
#include "kinemetry/config.hpp"

int main(int argc, char** argv) {
    std::cout << "kinemetry (kinecore " << kinecore::version() << ")\n";

    if (argc > 1) {
        const auto result = kinemetry::config::load_from_file(argv[1]);
        if (!result.ok) {
            std::cerr << "kinemetry: " << result.error << "\n";
            std::cerr << "kinemetry: falling back to defaults\n";
        } else {
            std::cout << "kinemetry: loaded config from " << argv[1] << "\n";
        }
        std::cout << result.config;
    } else {
        std::cout << "kinemetry: no config path given — using defaults\n";
        std::cout << kinemetry::config::Config{};
    }
    return 0;
}
