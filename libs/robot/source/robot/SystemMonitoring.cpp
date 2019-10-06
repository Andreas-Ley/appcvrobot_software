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

#include "SystemMonitoring.h"

#include "WifiCommunication.h"

#include "HardwareInterface.h"

#include <boost/format.hpp>

#include <chrono>
#include <iostream>
#include <fstream>
#include <sys/statvfs.h>
#include <sys/sysinfo.h>

#include <iostream>

namespace robot {
    
    
SystemMonitoring::SystemMonitoring(WifiCommunication *wifiCommunication) : m_wifiCommunication(wifiCommunication) {
    m_thread = std::thread([&]{
        
        unsigned lastTotalJiffies[5];
        unsigned lastWorkJiffies[5];
        
        auto readJiffies = [](unsigned *totalJiffies, unsigned *workJiffies) {
            std::fstream file("/proc/stat", std::fstream::in);
            std::string header;
            for (unsigned i = 0; i < 5; i++) {
                file >> header;
                unsigned user, nice, system, idle, iowait, irq, softirq, zero;
                file >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> zero >> zero >> zero;
                
                totalJiffies[i] = user + nice + system + idle + iowait + irq + softirq;
                workJiffies[i] = user + nice + system;
            }
        };
        
        readJiffies(lastTotalJiffies, lastWorkJiffies);
        
        while (!m_shutdown.load()) {
            
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            
            float cellVoltage1 = hardwareInterface::battery::getCellVoltage(hardwareInterface::battery::CELL_1);
            float batteryCurrentDraw = hardwareInterface::battery::getBatteryCurrentAmps();
            
            std::cout << "Cell voltage: " << cellVoltage1 << " V" << std::endl;
            std::cout << "Draw: " << batteryCurrentDraw << " A" << std::endl;
            
            
            unsigned temp = ~0u;
            {
                std::fstream file("/sys/class/thermal/thermal_zone0/temp", std::fstream::in);
                if (file)
                    file >> temp;
            }
            unsigned cpuFreq[4] = {~0u, ~0u, ~0u, ~0u};
            for (unsigned i = 0; i < 4; i++) {
                std::fstream file((boost::format("/sys/devices/system/cpu/cpu%d/cpufreq/cpuinfo_cur_freq") % i).str().c_str(), std::fstream::in);
                if (file) file >> cpuFreq[i];
            }
            
            unsigned totalJiffies[5];
            unsigned workJiffies[5];
            readJiffies(totalJiffies, workJiffies);

            float usage[5];
            for (unsigned i = 0; i < 5; i++) {
                usage[i] = workJiffies[i] / (float) totalJiffies[i];
                lastWorkJiffies[i] = workJiffies[i];
                lastTotalJiffies[i] = totalJiffies[i];
            }
            
            
            struct statvfs buf;
            memset(&buf, 0, sizeof(buf));
            statvfs("./", &buf);
            
            struct sysinfo sinfo;
            sysinfo(&sinfo);
            
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                for (unsigned i = 0; i < 4; i++)
                    m_state.cpuFreq_KHz[i] = cpuFreq[i];
                m_state.CPUTemp_mC = temp;
                for (unsigned i = 0; i < 5; i++)
                    m_state.cpuUsage[i] = usage[i] * 255;
                m_state.freeSpace_MB = (std::uint64_t) buf.f_bavail * (std::uint64_t) buf.f_bsize / 1024 / 1024;
                
                m_state.memTotal_KB = sinfo.totalram / 1024;
                m_state.memAvailable_KB = (sinfo.freeram + sinfo.bufferram) / 1024;
                
                m_state.cellVoltage_div20[0] = std::min<int>(std::max<int>(cellVoltage1 / 20.0f, 0), 255);
                m_state.batteryDrawAmps_div100 = std::min<int>(std::max<int>(batteryCurrentDraw / 100.0f, 0), 255);
            }
            
            if (m_wifiCommunication != nullptr) {
                
                WifiCommunication::Packet packet;
                packet.broadcast2Connections = true;
                packet.data.resize(sizeof(WifiCommunication::SystemMonitoringPacket));
                
                WifiCommunication::SystemMonitoringPacket *packetMon = (WifiCommunication::SystemMonitoringPacket *) packet.data.data();
                packetMon->header.messageType = WifiCommunication::MESSAGE_ID_SYSTEM_MONITORING;
                packetMon->header.sequenceNumber = m_nextSequenceNumber++;
                packetMon->state = m_state;
                
                m_wifiCommunication->send(std::move(packet), true);                
            }
        }
    });
}


SystemMonitoring::~SystemMonitoring()
{
    m_shutdown.store(true);
    m_thread.join();
}

}
