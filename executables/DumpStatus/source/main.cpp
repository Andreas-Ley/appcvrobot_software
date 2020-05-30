#include <robot/HardwareInterface.h>

#include <boost/thread/thread.hpp> 

#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

bool shutdown = false;

void handler(int s)
{
    shutdown = true;
}

int main(int argc, char **argv)
{

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);


    hardwareInterface::init();

    while (!shutdown) {

        std::cout << "Cells: " << hardwareInterface::battery::getCellVoltage(hardwareInterface::battery::CELL_1) << " "
                               << hardwareInterface::battery::getCellVoltage(hardwareInterface::battery::CELL_2) << " "
                               << hardwareInterface::battery::getCellVoltage(hardwareInterface::battery::CELL_3) << " ";

        std::cout << "Amps: " << hardwareInterface::battery::getBatteryCurrentAmps() << " ";

        std::cout << "Buttons: [";

        uint8_t buttons = hardwareInterface::getButtons();
        if (buttons & 1)
            std::cout << " power";
        if (buttons & 2)
            std::cout << " execute";
        if (buttons & 4)
            std::cout << " stop";
        std::cout << " ] ";

        std::cout << std::endl;

        boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
    }

    hardwareInterface::shutdown();

    return 0;
}
