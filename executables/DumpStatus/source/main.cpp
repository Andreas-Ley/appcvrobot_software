#include <controlSocketProtocol/RobotHardware.h>

#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>

#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

int main(int argc, char **argv)
{

    boost::asio::io_context ioContext;
    robot::RobotHardware robotHardware(ioContext);

    while (true)
    {
        robot::hardwareSocket::ResponseBodyCellVoltages responseBody = robotHardware.getBatteryCellVoltages();
        std::cout << "Cells: " << responseBody.voltages[0] << " "
                  << responseBody.voltages[1] << " "
                  << responseBody.voltages[2] << " ";
        std::cout << std::endl;

        boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
    }

    return 0;
}
