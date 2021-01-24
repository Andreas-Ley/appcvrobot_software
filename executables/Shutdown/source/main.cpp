#include <robot/HardwareInterface.h>
#include <iostream>


int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cout << "Requires argument" << std::endl;
        return 1;
    }

    if (std::string(argv[1]) == "poweroff") {
        hardwareInterface::init();

        hardwareInterface::battery::killPower();

        hardwareInterface::shutdown();
    }

    return 0;
}
