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

#ifndef SLAMSYSTEM_H
#define SLAMSYSTEM_H

#include "Subsystem.h"
#include "CameraSystem.h"

#include <System.h>

#include <Eigen/Dense>

#include <atomic>
#include <thread>
#include <mutex>

class WifiCommunication;

namespace robot {

class SLAMSystem : public Subsystem
{
    public:
        SLAMSystem(CameraSystem &camera, WifiCommunication *wifiCommunication = nullptr, unsigned reportWifiMaxRate = 0);
        
        virtual ~SLAMSystem();

        virtual void operateSlow(float dt);
        
        void getLastPose(Eigen::Matrix4f &pose, bool &valid);
        
        struct SLAMPoint {
            unsigned id;
            Eigen::Vector3f p, n;
            unsigned numObs;
            float foundRatio;
            std::uint8_t flags;
            
            enum {
                FLAG_IS_BAD             = (1 << 0),
                FLAG_IS_REFERENCE_POINT = (1 << 1),
            };
        };
        
        struct NetworkPose {
            float qx, qy, qz, qw; // Quaternion rotation
            float px, py, pz; // Position
            uint8_t trackingLost;
        } __attribute__((packed));

        struct NetworkMapSlice {
            unsigned numPointsTotal;
            unsigned sliceCount;
            unsigned sliceOffset;
            struct Point {
                std::uint32_t id;
                float x, y, z;
                std::int8_t nx, ny, nz, foundRatio;
                std::uint16_t numObs;
                std::uint8_t flags;
                std::uint8_t padding;
            } __attribute__((packed));
            Point points[0];
        } __attribute__((packed));
        
    protected:
        robot::CameraSystem &m_camera;
        std::atomic<bool> m_shutdown = std::atomic<bool>(false);
        std::thread m_slamThread;
        
        void mainSlamThread();
        
        ORB_SLAM2::System m_slam;

        std::mutex m_mutex;
        Eigen::Matrix4f m_lastEstimatedPose;
        bool m_lastPoseValid = false;
        std::vector<SLAMPoint> m_map;
        
        WifiCommunication *m_wifiCommunication = nullptr;
        unsigned m_reportWifiMaxRate = 0;
        unsigned m_nextUpdateOffset = 0;
        unsigned m_nextSequenceNumber = 0;
};


}


#endif // SLAMSYSTEM_H
