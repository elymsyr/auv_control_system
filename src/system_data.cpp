#include "system_data.h"
#include <arpa/inet.h>
#include <cstring>

std::vector<uint8_t> SystemData::serialize() {
    std::vector<uint8_t> buffer;
    
    // Header byte
    buffer.push_back('M');
    
    // Helper function to handle endianness and append data
    auto append_data = [&buffer](auto value) {
        using T = decltype(value);
        if constexpr (std::is_same_v<T, time_t>) {
            // Convert time_t to network byte order (64-bit)
            uint64_t net_value = htobe64(static_cast<uint64_t>(value));
            const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&net_value);
            buffer.insert(buffer.end(), bytes, bytes + sizeof(net_value));
        }
        else if constexpr (std::is_same_v<T, uint16_t>) {
            // CORRECTED: Use htons for 16-bit values
            uint16_t net_value = htons(value);
            const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&net_value);
            buffer.insert(buffer.end(), bytes, bytes + sizeof(net_value));
        }
    };
    
    // Serialize all available packets
    pack packet;
    while (packs.pop(packet)) {
        append_data(packet.timestamp);
        append_data(packet.code);
    }

    return buffer;
}

void SystemData::addPacket(uint8_t digit1, uint8_t digit2, uint8_t digit3, uint8_t digit4, uint8_t digit5) {
    uint16_t code = (digit1 * 10000) + (digit2 * 1000) + (digit3 * 100) + (digit4 * 10) + digit5;
    packs.push(pack(code));
}