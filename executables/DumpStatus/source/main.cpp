#include <controlSocketProtocol/ControlSocketProtocol.h>

#include <boost/thread/thread.hpp> 
#include <boost/asio.hpp>

#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

int main(int argc, char **argv)
{

    const char *socketFile = "/var/run/robothardwarecontroller.socket";
    using boostSockProt = boost::asio::local::stream_protocol;

    boost::asio::io_context ioContext;

    boostSockProt::socket socket(ioContext);
    socket.connect(boostSockProt::endpoint(socketFile));

    while (true) {
        robot::hardwareSocket::RequestCodes requestCode = robot::hardwareSocket::RequestCodes::BATTERY_CELL_VOLTAGES;
        struct {
            robot::hardwareSocket::ResponseCodes responseCode;
            robot::hardwareSocket::ResponseBodyCellVoltages body;
        } voltageResponse;
        memset(&voltageResponse, 0, sizeof(voltageResponse));     
        
        boost::asio::write(socket, boost::asio::buffer(&requestCode, sizeof(requestCode)));
        
        boost::asio::read(socket, boost::asio::buffer(&voltageResponse, sizeof(voltageResponse)));
        
        if (voltageResponse.responseCode == robot::hardwareSocket::ResponseCodes::OK) {
            std::cout << "Success" << std::endl;
        } else {
            std::cout << "Failure: " << (unsigned) voltageResponse.responseCode << std::endl;
            return -1;
        }

        std::cout << "Cells: " << voltageResponse.body.voltages[0] << " "
                               << voltageResponse.body.voltages[1] << " "
                               << voltageResponse.body.voltages[2] << " ";

//        std::cout << "Amps: " << hardwareInterface::battery::getBatteryCurrentAmps() << " ";
/*
        std::cout << "Buttons: [";
/*
        uint8_t buttons = hardwareInterface::getButtons();
        if (buttons & 1)
            std::cout << " power";
        if (buttons & 2)
            std::cout << " execute";
        if (buttons & 4)
            std::cout << " stop";
        std::cout << " ] ";
*/
        std::cout << std::endl;

        boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
    }

    return 0;
}
