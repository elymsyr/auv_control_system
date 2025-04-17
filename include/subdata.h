#ifndef BASE_DATA_H
#define BASE_DATA_H

#include <chrono>
#include <array>

class BaseData {
protected:
    double timing;
    int idx;

public:
    BaseData(int id = 0) : idx(id) {
        timing = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    // Remove the virtual keyword from the destructor
    ~BaseData() = default;

    double getTiming() const {
        return timing;
    }

    int getIdx() const {
        return idx;
    }
};

#endif // BASE_DATA_H