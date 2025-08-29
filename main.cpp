#include "uvc_gadget.hpp"
#include <iostream>
#include <string>
#include <getopt.h>

void usage(const char* prog) {
    std::cerr << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  -d, --device <device>   UVC device name (default: /dev/video0)\n"
              << "  -h, --help              Show this help message\n";
}

int main(int argc, char* argv[]) {
    std::string device_name = "/dev/video0";

    const char* const short_opts = "d:h";
    const option long_opts[] = {
        {"device", required_argument, nullptr, 'd'},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, 0, nullptr, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, short_opts, long_opts, nullptr)) != -1) {
        switch (opt) {
            case 'd':
                device_name = optarg;
                break;
            case 'h':
                usage(argv[0]);
                return 0;
            default:
                usage(argv[0]);
                return 1;
        }
    }

    // 仅支持 dummy 模式
    UvcGadget::UvcDevice uvc_device(device_name, true);

    if (!uvc_device.Open()) {
        return 1;
    }

    uvc_device.Run();

    return 0;
}