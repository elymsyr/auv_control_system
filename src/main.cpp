#include "system/main_system.h"
#include <iomanip>
#include <mutex>
#include <getopt.h>
#include <unistd.h>

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "  -H, --host <N>          Set host ip (default localhost)\n"
              << "  -d, --debug <M>         Set debug mode on (default false)\n"
              << "  -h, --help              Show this help\n";
}

int main(int argc, char** argv) {
    // Default values
    std::string host = "localhost";
    bool debug = false;
    bool help = false;

    const char* short_opts = "hH:d";
    const struct option long_opts[] = {
        {"help",  no_argument,        nullptr, 'h'},
        {"host",  required_argument,  nullptr, 'H'},
        {"debug", no_argument,        nullptr, 'd'},
        {nullptr, 0,                  nullptr,  0 }
    };

    int opt;
    while ((opt = getopt_long(argc, argv, short_opts, long_opts, nullptr)) != -1) {
        switch (opt) {
            case 'H':
                host = optarg;
                break;
            case 'd':
                debug = true;
                break;
            case 'h':
                help = true;
                break;
            case '?':
            default:
                print_usage(argv[0]);
                return EXIT_FAILURE;
        }
    }

    if (help) {
        print_usage(argv[0]);
        return EXIT_SUCCESS;
    }

    std::cout << "host = " << host
              << ", debug = " << std::boolalpha << debug << "\n";

    MainSystem mainSystem(host, debug);
    mainSystem.waitForDestruction();
    return 0;
}

// sudo ss -ltnp | grep -E ':(5560|5561|5562|5563|8889)\b'
// sudo fuser -k 5560/tcp 5561/tcp 5562/tcp 5563/tcp 8889/tcp