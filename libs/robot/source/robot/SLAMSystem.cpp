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

#include "SLAMSystem.h"
#include "WifiCommunication.h"

namespace robot {

SLAMSystem::SLAMSystem(robot::CameraSystem &camera, WifiCommunication *wifiCommunication, unsigned reportWifiMaxRate) :
        m_camera(camera),
        m_slam("Vocabulary/ORBvoc.txt", "../RPiCameraWide_640_480.yaml", ORB_SLAM2::System::MONOCULAR, true),
        m_wifiCommunication(wifiCommunication), 
        m_reportWifiMaxRate(reportWifiMaxRate)
{
    
    m_lastEstimatedPose.setIdentity();
    
    m_slamThread = std::thread([&]{ mainSlamThread(); });
}

SLAMSystem::~SLAMSystem() 
{
    m_shutdown.store(true);
    m_slamThread.join();
}
        
void SLAMSystem::mainSlamThread() 
{
    std::chrono::time_point<std::chrono::steady_clock> startTime = std::chrono::steady_clock::now();
    
    std::vector<SLAMPoint> newMap;
    
    while (!m_shutdown.load()) {
        auto t = std::chrono::steady_clock::now();
        double timestamp = std::chrono::duration_cast<std::chrono::duration<double> >(t - startTime).count();


        auto a = std::chrono::steady_clock::now();
        Eigen::Matrix4f pose;
        bool valid = m_slam.TrackMonocular(m_camera.getCurrentFrame(), timestamp, pose);
        auto b = std::chrono::steady_clock::now();
        std::cout << "SLAM.TrackMonocular frame took " << std::chrono::duration_cast<std::chrono::duration<double> >(b - a).count() << " seconds" << std::endl;
        
        std::set<ORB_SLAM2::MapPoint*> referencePoints;
        {
            const std::vector<ORB_SLAM2::MapPoint*> vpRefMPs = m_slam.getMap()->GetReferenceMapPoints();
            for (auto p : vpRefMPs)
                referencePoints.insert(p);
        }
        
        const std::vector<ORB_SLAM2::MapPoint*> vpMPs = m_slam.getMap()->GetAllMapPoints();
        newMap.resize(vpMPs.size());
        for (unsigned i = 0; i < vpMPs.size(); i++) {
            auto &dst = newMap[i];
            auto &src = *vpMPs[i];
            
            bool isBad;
            src.getInfo(dst.id, dst.p, dst.n, isBad, dst.numObs, dst.foundRatio);
            
            dst.flags = 0;
            if (referencePoints.find(&src) != referencePoints.end())
                dst.flags |= SLAMPoint::FLAG_IS_REFERENCE_POINT;
            if (isBad)
                dst.flags |= SLAMPoint::FLAG_IS_BAD;
        }
        
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (valid)
                m_lastEstimatedPose = pose;
            m_lastPoseValid = valid;
            std::swap(m_map, newMap);
        }
    }
}

void SLAMSystem::operateSlow(float dt)
{ 
    if ((m_wifiCommunication != nullptr) && (m_reportWifiMaxRate > 0)) {
        
        int bytesRemaining = std::ceil(m_reportWifiMaxRate * dt);
        
        WifiCommunication::Packet packet;

        
    
        packet.data.resize(sizeof(WifiCommunication::SLAMPosePacket));
        
        WifiCommunication::SLAMPosePacket *slamPosePacket = (WifiCommunication::SLAMPosePacket*)packet.data.data();
        
        slamPosePacket->header.messageType = WifiCommunication::MESSAGE_ID_SLAM_POSE;
        slamPosePacket->header.sequenceNumber = m_nextSequenceNumber++;
        
        {
            Eigen::Matrix4f pose;
            bool valid;
            getLastPose(pose, valid);
            
            Eigen::Quaternionf rot(pose.block<3, 3>(0, 0));                
            slamPosePacket->pose.qx = rot.x();
            slamPosePacket->pose.qy = rot.y();
            slamPosePacket->pose.qz = rot.z();
            slamPosePacket->pose.qw = rot.w();
            slamPosePacket->pose.px = pose(0, 3);
            slamPosePacket->pose.py = pose(1, 3);
            slamPosePacket->pose.pz = pose(2, 3);
            slamPosePacket->pose.trackingLost = valid?0:1;
        }
        
        packet.broadcast2Connections = true;
        
        bytesRemaining -= packet.data.size();
        m_wifiCommunication->send(std::move(packet), true);

        std::lock_guard<std::mutex> lock(m_mutex);
        while (bytesRemaining > 0) {
            const unsigned maxSliceSize = (WifiCommunication::MAX_PACKET_SIZE - sizeof(WifiCommunication::SLAMMapSlicePacket)) / sizeof(NetworkMapSlice::Point);
            
            unsigned sliceSize = std::min<unsigned>(maxSliceSize, m_map.size());
            if (m_map.size() > 0)
                m_nextUpdateOffset = m_nextUpdateOffset % m_map.size();
            else
                m_nextUpdateOffset = 0;
            
            packet.data.resize(sizeof(WifiCommunication::SLAMMapSlicePacket) + sizeof(NetworkMapSlice::Point)*sliceSize);

            WifiCommunication::SLAMMapSlicePacket *slamMapSlicePacket = (WifiCommunication::SLAMMapSlicePacket*)packet.data.data();
            
            slamMapSlicePacket->header.messageType = WifiCommunication::MESSAGE_ID_SLAM_MAP_SLICE;
            slamMapSlicePacket->header.sequenceNumber = m_nextSequenceNumber++;

            slamMapSlicePacket->mapSlice.numPointsTotal = m_map.size();
            slamMapSlicePacket->mapSlice.sliceCount = sliceSize;
            slamMapSlicePacket->mapSlice.sliceOffset = m_nextUpdateOffset;
            for (unsigned i = 0; i < sliceSize; i++) {
                auto &p = slamMapSlicePacket->mapSlice.points[i];
                p.id = m_map[m_nextUpdateOffset].id;
                p.x = m_map[m_nextUpdateOffset].p[0];
                p.y = m_map[m_nextUpdateOffset].p[1];
                p.z = m_map[m_nextUpdateOffset].p[2];
                p.nx = m_map[m_nextUpdateOffset].n[0] * 127.0f;
                p.ny = m_map[m_nextUpdateOffset].n[1] * 127.0f;
                p.nz = m_map[m_nextUpdateOffset].n[2] * 127.0f;
                p.foundRatio = m_map[m_nextUpdateOffset].foundRatio * 255.0f;
                p.numObs = m_map[m_nextUpdateOffset].numObs;
                p.flags = m_map[m_nextUpdateOffset].flags;
                p.padding = 0;

                m_nextUpdateOffset++;
                if (m_nextUpdateOffset == m_map.size())
                    m_nextUpdateOffset = 0;
            }
            
            packet.broadcast2Connections = true;
            
            bytesRemaining -= packet.data.size();
            m_wifiCommunication->send(std::move(packet), false);
        }
    }
}

void SLAMSystem::getLastPose(Eigen::Matrix4f &pose, bool &valid) {
    std::lock_guard<std::mutex> lock(m_mutex);
    pose = m_lastEstimatedPose;
    valid = m_lastPoseValid;
}

#if 0
void renderMap(const char *filename) {
    if (m_slam.getMap() != nullptr) {
        const std::vector<ORB_SLAM2::MapPoint*> vpMPs = m_slam.getMap()->GetAllMapPoints();
        const std::vector<ORB_SLAM2::MapPoint*> vpRefMPs = m_slam.getMap()->GetReferenceMapPoints();
        
        unsigned width = 1024;
        unsigned height = 1024;
        cv::Mat image = cv::Mat::zeros(width, height, CV_8UC3 );
        
        Eigen::Matrix<float, 2, 3> world2imageScale;
        world2imageScale.setZero();
        world2imageScale(0, 0) = width/4.0f;
        world2imageScale(1, 2) = width/4.0f;
        Eigen::Vector2f world2imageOffset(width/2, height/2);
        
        for (auto p : vpMPs) {
            if (p->isBad()) continue;
            unsigned numObs = p->Observations();
            Eigen::Vector3f pos = p->GetWorldPos();
            Eigen::Vector3f normal = p->GetNormal();
            
            Eigen::Vector2f imgPos = world2imageOffset + world2imageScale * pos;
            Eigen::Vector2f imgNormal = world2imageScale * normal;
            imgNormal.normalize();
            
            int x = imgPos[0] + 0.5f;
            int y = imgPos[1] + 0.5f;
            
            if ((x < 0) || (x >= width) ||
                (y < 0) || (y >= height)) continue;
            
            unsigned color_r = 128 + std::min<int>(127, numObs * 20);
            unsigned color_g = 128 + std::max<int>(-128, std::min<int>(127, pos[2] * 200));
            image.at<cv::Vec3b>(y, x)[0] = color_r;
            image.at<cv::Vec3b>(y, x)[1] = color_g;
            image.at<cv::Vec3b>(y, x)[2] = 0;
            /*
            cv::line(image,
                        cv::Point(x, y),
                        cv::Point(x + imgNormal[0] * 5, y + imgNormal[1] * 5),
                        cv::Scalar( 128, 0, 0 ),
                        1,
                        cv::LINE_4);

            */
        }
        unsigned circleSize = 0.3f * world2imageScale(0, 0);
        Eigen::Vector2f targetImgPos = world2imageOffset + world2imageScale * target;
        cv::circle(image,
                    cv::Point(targetImgPos[0], targetImgPos[1]),
                    circleSize,
                    cv::Scalar( 0, 0, 256 ),
                    1,
                    cv::LINE_4);

        for (auto p : waypoints) {
            Eigen::Vector2f imgP = world2imageOffset + world2imageScale * p;
            cv::circle(image,
                        cv::Point(imgP[0], imgP[1]),
                        circleSize,
                        cv::Scalar( 0, 256, 256 ),
                        1,
                        cv::LINE_4);
        }

        
        {
            Eigen::Matrix4f pose;
            bool poseValid;
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                pose = m_lastEstimatedPose;
                poseValid = m_lastPoseValid;
            }

            if (poseValid) {
                Eigen::Vector3f camPoints[3];
                camPoints[0] = Eigen::Vector3f(-pose(0, 3), -pose(1, 3), -pose(2, 3));
                float angle = std::atan2(-pose(2, 0), -pose(2, 2));
                float size = 0.1f;
                float fov_half = 160 / 2.0f / 180.0f * M_PI;
                camPoints[1] = camPoints[0] + Eigen::Vector3f(-std::sin(angle - fov_half) * size, 0.0f, -std::cos(angle - fov_half) * size);
                camPoints[2] = camPoints[0] + Eigen::Vector3f(-std::sin(angle + fov_half) * size, 0.0f, -std::cos(angle + fov_half) * size);
                
                Eigen::Vector2f camImgPoints[3];                
                for (unsigned i = 0; i < 3; i++)
                    camImgPoints[i] = world2imageOffset + world2imageScale * camPoints[i];
                
                cv::line(image,
                            cv::Point(camImgPoints[0][0], camImgPoints[0][1]),
                            cv::Point(camImgPoints[1][0], camImgPoints[1][1]),
                            cv::Scalar( 0, 0, 128 ),
                            1,
                            cv::LINE_4);
                cv::line(image,
                            cv::Point(camImgPoints[0][0], camImgPoints[0][1]),
                            cv::Point(camImgPoints[2][0], camImgPoints[2][1]),
                            cv::Scalar( 0, 0, 128 ),
                            1,
                            cv::LINE_4);
                cv::line(image,
                            cv::Point(camImgPoints[2][0], camImgPoints[2][1]),
                            cv::Point(camImgPoints[1][0], camImgPoints[1][1]),
                            cv::Scalar( 0, 0, 128 ),
                            1,
                            cv::LINE_4);

            }
        }
        cv::imwrite(filename, image);
    } else {
        unsigned width = 1024;
        unsigned height = 1024;
        cv::Mat image = cv::Mat::zeros(width, height, CV_8UC3 );
        cv::imwrite(filename, image);
    }
}
#endif

}
