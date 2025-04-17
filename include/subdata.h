#ifndef BASE_DATA_H
#define BASE_DATA_H

#include <chrono>
#include <array>

class BaseData {
protected:
    double timing;
public:
    BaseData() {
        timing = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    // Remove the virtual keyword from the destructor
    ~BaseData() = default;

    double getTiming() const {
        return timing;
    }
};

#endif // BASE_DATA_H