#include <iostream>

#include "kinecore/version.hpp"

int main() {
    std::cout << "kinemetry (kinecore " << kinecore::version() << ")\n";
    return 0;
}
