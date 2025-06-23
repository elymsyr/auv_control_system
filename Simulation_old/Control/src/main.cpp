#include "main_system.h"

int main() {
    MainSystem mainSystem;
    mainSystem.waitForDestruction();
    return 0;
}

// sudo ss -ltnp | grep -E ':(5560|5561|5562|5563|8889)\b'
// sudo fuser -k 5560/tcp 5561/tcp 5562/tcp 5563/tcp 8889/tcp