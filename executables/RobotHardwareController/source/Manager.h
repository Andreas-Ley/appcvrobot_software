#ifndef MANAGER_H
#define MANAGER_H

/**
 * @todo write docs
 */

#include <boost/asio.hpp>

class Session;

class Manager
{
    public:
        enum {
            CHECK_BUTTON_INTERVAL = 2000,
            CHECK_BATTERY_INTERVAL = 10000
        };
        
        enum Lock {
            MOTOR, DISPLAY, LOCK_COUNT
        };
        
        Manager(boost::asio::io_context& ioContext);
        
        bool tryAcquireLock(Lock lock, Session *session);
        void releaseLock(Lock lock);
        bool isLockedBy(Lock lock, Session *session);

        void checkButtons();
        void checkBatteries();
    protected:
        boost::asio::io_context& m_ioContext;
        
        Session* m_lock[LOCK_COUNT] = { };
        
        boost::asio::deadline_timer m_buttonTimer;
        boost::asio::deadline_timer m_batteryTimer;
};

#endif // MANAGER_H
