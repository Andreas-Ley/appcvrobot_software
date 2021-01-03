#include "HardwareInterface.h"
#include "ControlSocket.h"
#include "config.h"
#include "Logger.h"
#include "Manager.h"

#include <fstream>
#include <iostream>
#include <cstdio>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

void shutdown() 
{
    logger.log(0)<< "Shutting down RobotHardwareController" << std::endl;
    hardwareInterface::shutdown();
}

int main()
{
    logger.log(0) << "Starting up RobotHardwareController" << std::endl;


    
    try {
        logger.log(0)  << "Initializing gpio" << std::endl;
        hardwareInterface::init();//start I2C
        //atexit(shutdown); // execute when program terminates

        //system((std::string("rm ") + socketFile).c_str()); // todo: look for better option
        unlink(socketFile);
		
        boost::asio::io_context ioContext;
        Manager manager(ioContext);

        ControlSocket ControlSocket(ioContext, manager);

        ioContext.run();
    } catch (const std::exception &e) {
        logger.log(0)  << "An exception occured: " << e.what() << std::endl;
    }
        
    return 0;
}
