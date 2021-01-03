#include "Manager.h"
#include "Logger.h"

#include "HardwareInterface.h"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

Manager::Manager(boost::asio::io_context& ioContext):
                    m_ioContext(ioContext), 
                    m_buttonTimer(ioContext, boost::posix_time::seconds(0)),
                    m_batteryTimer(ioContext, boost::posix_time::seconds(0))
{
    m_buttonTimer.async_wait(boost::bind(&Manager::checkButtons, this));
    m_batteryTimer.async_wait(boost::bind(&Manager::checkBatteries, this));

}

bool Manager::tryAcquireLock(Lock lock, Session *session){
    if (m_lock[lock] != nullptr){
        return false;
    }
    
    m_lock[lock] = session;
    return true;
    
}
void Manager::releaseLock(Lock lock){

    m_lock[lock] = nullptr;
}


bool Manager::isLockedBy(Lock lock, Session *session){
    return m_lock[lock] == session;
}
        
void Manager::checkButtons()
{
    unsigned buttons = hardwareInterface::getButtons();
    
    if (buttons & 1) {
        logger.log(0) << "Power button pressed, initiating shutdown!" << std::endl;
        
        system("shutdown now");
    }
    
    m_buttonTimer.expires_at(m_buttonTimer.expires_at() + boost::posix_time::millisec((unsigned)CHECK_BUTTON_INTERVAL));
    m_buttonTimer.async_wait(boost::bind(&Manager::checkButtons, this));
   
}
void Manager::checkBatteries(){
    
    m_batteryTimer.expires_at(m_batteryTimer.expires_at() + boost::posix_time::millisec((unsigned)CHECK_BATTERY_INTERVAL));
    m_batteryTimer.async_wait(boost::bind(&Manager::checkBatteries, this));
}
