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

#ifndef CAMERASYSTEM_H
#define CAMERASYSTEM_H

#include "Subsystem.h"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <atomic>
#include <mutex>
#include <thread>

namespace robot {

class WifiCommunication;
    
class CameraSystem : public Subsystem
{
    public:
        CameraSystem(WifiCommunication *wifiCommunication = nullptr, unsigned reportWifiMaxRate = 0);
        virtual ~CameraSystem();

        virtual void operateSlow(float dt) override;
        
        cv::Mat getCurrentFrame();
    protected:
        std::atomic<bool> m_shutdown = std::atomic<bool>(false);
        std::mutex m_mutex;
        std::thread m_captureThread;
        cv::VideoCapture m_videoCapture;
        cv::Mat m_currentFrame;
        cv::Mat m_lastFrame;
        cv::Mat m_wifiSubmittedFrame;
        
        WifiCommunication *m_wifiCommunication = nullptr;
        unsigned m_reportWifiMaxRate = 0;
        
        enum {
            WIFI_TILE_SIZE = 64
        };
        
        unsigned m_nextCameraTileSequenceNumber = 0;
        
};

}

#endif // CAMERASYSTEM_H
