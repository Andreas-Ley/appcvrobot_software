#include <controlSocketProtocol/RobotHardware.h>
//#include <robot/HardwareInterface.h>

#include <boost/asio.hpp>

#include <iostream>
#include <string.h>

int main(int argc, char **argv)
{

#if 1

    boost::asio::io_context ioContext;
    robot::RobotHardware robotHardware(ioContext);

    if (argc < 2)
    {
        robotHardware.setLcdText("Hello", "    World!!!");
    }
    else
    {
        if (argc > 2)
            robotHardware.setLcdText(argv[1], argv[2]);
        else
            robotHardware.setLcdText(argv[1], "");
    }

#else
    hardwareInterface::init();

    hardwareInterface::lcd::initDisplay();
    hardwareInterface::lcd::clear();

    if (argc < 2)
    {
        hardwareInterface::lcd::writeLine("Hello", 0);
        hardwareInterface::lcd::writeLine("   World!!!", 1);
    }
    else
    {
        hardwareInterface::lcd::writeLine(argv[1], 0);
        if (argc > 2)
            hardwareInterface::lcd::writeLine(argv[2], 1);
    }

    hardwareInterface::shutdown();

    return 0;
#endif
}
