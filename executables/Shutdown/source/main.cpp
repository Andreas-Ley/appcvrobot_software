#include <robot/HardwareInterface.h>
#include <iostream>


int main(int argc, char **argv)
{
    hardwareInterface::init();

    hardwareInterface::battery::killPower();

    hardwareInterface::shutdown();

    return 0;
}
