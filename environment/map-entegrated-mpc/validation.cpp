#include <fstream>
#include <iostream>
#include "EnvironmentMap.h"
#include <cmath> 

uint8_t* read_grid(const char* filename, int w, int h) {
    uint8_t* grid = new uint8_t[w * h];
    std::ifstream file(filename, std::ios::binary);
    file.read(reinterpret_cast<char*>(grid), w * h);
    return grid;
}

bool verify_shift(const char* before, const char* after, int shiftX, int shiftY) {
    const int W = 129, H = 129;
    uint8_t* grid_before = read_grid(before, W, H);
    uint8_t* grid_after = read_grid(after, W, H);
    
    bool valid = true;
    for(int y = 0; y < H; y++) {
        for(int x = 0; x < W; x++) {
            int srcX = x - shiftX;
            int srcY = y - shiftY;
            
            uint8_t expected = (srcX >= 0 && srcX < W && srcY >= 0 && srcY < H) 
                             ? grid_before[srcY * W + srcX] 
                             : 0;
            
            if(grid_after[y * W + x] != expected) {
                valid = false;
                break;
            }
        }
        if(!valid) break;
    }
    
    delete[] grid_before;
    delete[] grid_after;
    return valid;
}

int main() {
    const char* before = "test_initial.bin";
    const char* after = "test_shifted.bin";
    int shiftX = 2, shiftY = 1;
    
    bool result = verify_shift(before, after, shiftX, shiftY);
    std::cout << "Shift validation: " << (result ? "PASSED" : "FAILED") << std::endl;
    return 0;
}