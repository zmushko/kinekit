#include <iostream>

#include "kinecore/version.hpp"

int main() {
    std::cout << "kinegram (kinecore " << kinecore::version() << ")\n";
    return 0;
}
