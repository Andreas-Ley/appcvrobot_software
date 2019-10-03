#include <robot/HardwareInterface.h>
#include <iostream>


int main(int argc, char **argv)
{
    hardwareInterface::init();

    hardwareInterface::lcd::initDisplay();
    hardwareInterface::lcd::clear();

    if (argc < 2) {
        hardwareInterface::lcd::writeLine("Hello", 0);
        hardwareInterface::lcd::writeLine("   World!!!", 1);
    } else {
        hardwareInterface::lcd::writeLine(argv[1], 0);
        if (argc > 2)
            hardwareInterface::lcd::writeLine(argv[2], 1);
    }

    hardwareInterface::shutdown();

    return 0;
}
