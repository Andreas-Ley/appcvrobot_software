#ifndef ROBOTHARDWARE_H
#define ROBOTHARDWARE_H

#include "ControlSocketProtocol.h"

#include <boost/asio.hpp>

#include <mutex>


namespace robot {   
    

/**
 * @todo write docs
 */
class RobotHardware
{
    public:
        RobotHardware(boost::asio::io_context& ioContext);
        hardwareSocket::ResponseBodyCellVoltages getBatteryCellVoltages();
        float getCurrentDraw();
        float getControllerUsage();
        void acquireDrive();
        void releaseDrive();
        void setDriveSpeed(float left, float right);
        std::tuple<int, int> getDriveSteps();
        void setLcdText(const std::string& s1, const std::string& s2);
        void acquireLcd();
        void releaseLcd();
        hardwareSocket::ResponseBodyButtons getButtonsPushed();       
        
    protected:
        using boostSockProt = boost::asio::local::stream_protocol;
        boostSockProt::socket m_socket;
        std::mutex m_mutex;
};

}

#endif // ROBOTHARDWARE_H
