#ifndef SYSTEM_DATA_H
#define SYSTEM_DATA_H

#include <vector>
#include <ctime>
#include <cstdint>
#include <chrono>
#include <boost/lockfree/spsc_queue.hpp>

class SystemData {
public:
    double timer = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch())
        .count();
    
    void reset_timer() {
        timer = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    struct pack {
        uint16_t code;
        std::time_t timestamp;
        pack(uint16_t code = 0) 
            : code(code), timestamp(std::time(nullptr)) {}
    };

    std::vector<uint8_t> serialize();
    void addPacket(uint8_t digit1, uint8_t digit2, uint8_t digit3, uint8_t digit4, uint8_t digit5);

protected:
    // No mutex needed - spsc_queue is lock-free for 1 producer/1 consumer
    boost::lockfree::spsc_queue<pack> packs{128};
};

#endif // SYSTEM_DATA_H