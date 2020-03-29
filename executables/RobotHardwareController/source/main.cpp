#include "HardwareInterface.h"
#include "ControlSocket.h"
#include "config.h"

#include <fstream>
#include <iostream>
#include <cstdio>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

std::fstream logfile;

void shutdown() 
{
    logfile << "Shutting down RobotHardwareController" << std::endl;
    hardwareInterface::shutdown();
}

void runPeriodicChecks(boost::asio::deadline_timer &timer, const boost::system::error_code&) 
{
    std::cout << "example how to run periodic checks!" << std::endl;
    
    timer.expires_at(timer.expires_at() + boost::posix_time::millisec(5000)); // rather large for now so the text output is not too annoying
    timer.async_wait(boost::bind(&runPeriodicChecks, boost::ref(timer), boost::asio::placeholders::error));
}

int main()
{
    logfile.open(logFile, std::fstream::app);
    logfile << "Starting up RobotHardwareController" << std::endl;


    
    try {
        logfile << "Initializing gpio" << std::endl;
        hardwareInterface::init();//start I2C
        atexit(shutdown); // execute when program terminates

        //system((std::string("rm ") + socketFile).c_str()); // todo: look for better option
        unlink(socketFile);
		
        boost::asio::io_context ioContext;


        ControlSocket ControlSocket(ioContext);
        
        boost::asio::deadline_timer timer(ioContext, boost::posix_time::seconds(0));
        timer.async_wait(boost::bind(&runPeriodicChecks, boost::ref(timer), boost::asio::placeholders::error));

        ioContext.run();
        
    } catch (const std::exception &e) {
        logfile << "An exception occured: " << e.what() << std::endl;
    }


    return 0;
}
