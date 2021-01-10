#include <controlSocketProtocol/ControlSocketProtocol.h>
//#include <robot/HardwareInterface.h>


#include <boost/asio.hpp>


#include <iostream>
#include <string.h>


int main(int argc, char **argv)
{
    
#if 1
    const char *socketFile = "/var/run/robothardwarecontroller.socket";
    using boostSockProt = boost::asio::local::stream_protocol;

    boost::asio::io_context ioContext;

    boostSockProt::socket socket(ioContext);
    socket.connect(boostSockProt::endpoint(socketFile));

    struct {
        robot::hardwareSocket::RequestCodes requestCode;
        robot::hardwareSocket::RequestBodyLCDSetText body;
    } request;
    memset(&request, 0, sizeof(request));
    
    request.requestCode = robot::hardwareSocket::RequestCodes::LCD_SET_TEXT;
    if (argc < 2) {
        request.body.setLine(0, "Hello");
        request.body.setLine(1, "    World!!!");
    } else {
        request.body.setLine(0, argv[1]);
        if (argc > 2)
            request.body.setLine(1, argv[2]);
    }
    
    
    boost::asio::write(socket, boost::asio::buffer(&request, sizeof(request)));
    
    robot::hardwareSocket::ResponseCodes response;
    boost::asio::read(socket, boost::asio::buffer(&response, sizeof(response)));
    
    if (response == robot::hardwareSocket::ResponseCodes::OK) {
        std::cout << "Success" << std::endl;
        return 0;
    } else {
        std::cout << "Failure: " << (unsigned) response << std::endl;
        return -1;
    }
    
#else
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
#endif
}
