#include <vector>
#include <string>
#include <iostream>
#include <fstream>

int main() {
    std::vector<std::string> test_vector = {"3", "4", "0", "1", "2"};
    for (int i = 0; i < 10; ++i) {
        std::cout << "Iteration: " << i << std::endl;
        for (const auto& test : test_vector) {
            std::string system_str = "./mpc_test" + test;
            std::cout << "  Running: " << system_str << std::endl;
            if (!std::ifstream("mpc_test" + test)) {
                std::cout << "  Generating: " << system_str << std::endl;
                // Generate derivatives explicitly
                std::string command = "g++ -std=c++17 -I\"${CONDA_PREFIX}/include\" -L\"${CONDA_PREFIX}/lib\" -Wl,-rpath,\"${CONDA_PREFIX}/lib\" mpc" + test + ".cpp -lcasadi -lipopt -lzmq -o mpc_test" + test;
                std::system(command.c_str());
            }
            std::system (system_str.c_str());
        }
    }
    return 0;
}

// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -L"${CONDA_PREFIX}/lib" -Wl,-rpath,"${CONDA_PREFIX}/lib" mpc0.cpp -lcasadi -lipopt -lzmq -o mpc_test0