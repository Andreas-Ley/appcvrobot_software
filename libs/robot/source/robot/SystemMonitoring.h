/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2019  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SYSTEMMONITORING_H
#define SYSTEMMONITORING_H

#include "Subsystem.h"

#include <controlSocketProtocol/RobotHardware.h>

#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>

namespace robot {

class WifiCommunication;
    
class SystemMonitoring : public Subsystem
{
    public:
        SystemMonitoring(WifiCommunication *wifiCommunication = nullptr);
        virtual ~SystemMonitoring();

        struct State {
            std::uint32_t CPUTemp_mC = 0;
            std::uint32_t freeSpace_MB = 0;
            std::uint32_t memTotal_KB = 0;
            std::uint32_t memAvailable_KB = 0;
            std::uint32_t cpuFreq_KHz[4] = {};
            std::uint8_t cpuUsage[5] = {};
            std::uint8_t cellVoltage_div20[3] = {};
            std::uint8_t batteryDrawAmps_div100 = 0;
        };
        
        
        State getState() const { 
            std::lock_guard<std::mutex> lock(m_mutex);
            return m_state;
        }
    protected:
        WifiCommunication *m_wifiCommunication;
        std::atomic<bool> m_shutdown = std::atomic<bool>(false);
        mutable std::mutex m_mutex;
        std::thread m_thread;    
        State m_state;
        
        unsigned m_nextSequenceNumber = 0;
};

}

#endif // SYSTEMMONITORING_H
