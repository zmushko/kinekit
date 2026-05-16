#include <iostream>

#include "kinecore/version.hpp"
#include "kinegram/config.hpp"

int main(int argc, char** argv) {
    std::cout << "kinegram (kinecore " << kinecore::version() << ")\n";

    if (argc > 1) {
        const auto result = kinegram::config::load_from_file(argv[1]);
        if (!result.ok) {
            std::cerr << "kinegram: " << result.error << "\n";
            std::cerr << "kinegram: falling back to defaults\n";
        } else {
            std::cout << "kinegram: loaded config from " << argv[1] << "\n";
        }
        std::cout << result.config;
    } else {
        std::cout << "kinegram: no config path given — using defaults\n";
        std::cout << kinegram::config::Config{};
    }
    return 0;
}
