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

#include "CameraSystem.h"

#include "WifiCommunication.h"

#include <opencv2/imgcodecs.hpp>

#include <stdio.h>
#include <iostream>

namespace robot {

CameraSystem::CameraSystem(WifiCommunication *wifiCommunication, unsigned reportWifiMaxRate) : m_wifiCommunication(wifiCommunication), m_reportWifiMaxRate(reportWifiMaxRate) {
    system("v4l2-ctl -c iso_sensitivity_auto=0");
    system("v4l2-ctl -c iso_sensitivity=4");
    system("v4l2-ctl -c scene_mode=11");
    system("v4l2-ctl -c video_bitrate=25000000");
    system("v4l2-ctl --set-fmt-video=width=640,height=480,pixelformat=2");
    if (!m_videoCapture.open(0)) {
        throw std::runtime_error("Error opening video");
    }
    
    m_captureThread = std::thread([&]{
        cv::Mat frame;
        while (!m_shutdown.load()) {
            m_videoCapture >> frame;
            
            std::lock_guard<std::mutex> lock(m_mutex);
            cv::Mat temp = m_lastFrame;
            m_lastFrame = m_currentFrame;
            m_currentFrame = frame;
            frame = temp;
        }
    });
}

CameraSystem::~CameraSystem() {
    m_shutdown.store(true);
    m_captureThread.join();
}


cv::Mat CameraSystem::getCurrentFrame() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_currentFrame.clone();
}


void CameraSystem::operateSlow(float dt)
{
    if (m_wifiCommunication != nullptr && m_reportWifiMaxRate > 0) {

        int bytesRemaining = dt * m_reportWifiMaxRate;
//std::cout << "Sending " << bytesRemaining << " bytes of camera data" << std::endl;
        std::lock_guard<std::mutex> lock(m_mutex);

        m_currentFrame.convertTo(m_currentFrameF, CV_32FC3);
        
        if (m_wifiSubmittedFrame.empty() || (m_wifiSubmittedFrame.rows != m_currentFrame.rows) || (m_wifiSubmittedFrame.cols != m_currentFrame.cols)) {
            m_wifiSubmittedFrame = m_currentFrameF.clone();
        }
        
        std::vector<std::tuple<float, int, int>> tiles;

        float totalChange = 0.0f;
        
        unsigned tileRows = (m_currentFrameF.rows + WIFI_TILE_SIZE-1) / WIFI_TILE_SIZE;
        unsigned tileCols = (m_currentFrameF.cols + WIFI_TILE_SIZE-1) / WIFI_TILE_SIZE;
        cv::Mat diff = m_currentFrameF - m_wifiSubmittedFrame;
        for (unsigned r = 0; r < tileRows; r++)
            for (unsigned c = 0; c < tileCols; c++) {
                auto m = cv::mean(cv::abs(
                    diff
                        .colRange(c*WIFI_TILE_SIZE, std::min<int>(diff.cols, (c+1)*WIFI_TILE_SIZE))
                        .rowRange(r*WIFI_TILE_SIZE, std::min<int>(diff.rows, (r+1)*WIFI_TILE_SIZE))
                                        ));
                float change = m[0] + m[1] + m[2];
                totalChange += change;
                if (change > 0.0f)
                    tiles.push_back({change, c, r});
            }
            
        std::sort(tiles.begin(), tiles.end());
        
        float changeUpdated = 0.0f;
        
        unsigned maxTileSize = 0;
        
        std::vector<unsigned char> imgBuffer;
        WifiCommunication::Packet packet;
        while (bytesRemaining > 0 && !tiles.empty()) {
            float s = std::get<0>(tiles.back());
            changeUpdated += s;
            int c = std::get<1>(tiles.back());
            int r = std::get<2>(tiles.back());
            tiles.pop_back();
            
            int x1 = c*WIFI_TILE_SIZE;
            int y1 = r*WIFI_TILE_SIZE;
            int x2 = std::min<int>(m_currentFrame.cols, (c+1)*WIFI_TILE_SIZE);
            int y2 = std::min<int>(m_currentFrame.rows, (r+1)*WIFI_TILE_SIZE);
            
            cv::Mat tile = m_currentFrame
                            .colRange(x1, x2)
                            .rowRange(y1, y2);
                            
            m_currentFrameF 
                        .colRange(x1, x2)
                        .rowRange(y1, y2)
                        .copyTo(
                            m_wifiSubmittedFrame
                                .colRange(x1, x2)
                                .rowRange(y1, y2)
                         );
                            
            cv::imencode(".jpg", tile, imgBuffer, {cv::IMWRITE_JPEG_QUALITY, m_wifiCompression, cv::IMWRITE_JPEG_PROGRESSIVE, 0, cv::IMWRITE_JPEG_OPTIMIZE, 0});
//std::cout << "Compressed tile with " << imgBuffer.size()*8.0f/((x2-x1) * (y2-y1)) << "bpp" << std::endl;
            
            packet.data.resize(sizeof(WifiCommunication::CameraTilePacket) + imgBuffer.size());
            
            WifiCommunication::CameraTilePacket *camTilePacket = (WifiCommunication::CameraTilePacket*)packet.data.data();
            
            camTilePacket->header.messageType = WifiCommunication::MESSAGE_ID_CAMERA_TILE;
            camTilePacket->header.sequenceNumber = m_nextCameraTileSequenceNumber++;
            camTilePacket->tileSpec = {
                m_currentFrame.cols, m_currentFrame.rows,
                x1, y1, x2-x1, y2-y1
            };
            memcpy(camTilePacket->data, imgBuffer.data(), imgBuffer.size());

            packet.broadcast2Connections = true;
            
            bytesRemaining -= packet.data.size();
            maxTileSize = std::max<unsigned>(maxTileSize, packet.data.size());
            
            m_wifiCommunication->send(std::move(packet), false);

        }
        
        if ((changeUpdated > totalChange * 0.8f) && (maxTileSize < WifiCommunication::MAX_PACKET_SIZE)) {
            m_wifiCompression = std::min<unsigned>(100, m_wifiCompression+1);
        } else {
            m_wifiCompression = std::max<unsigned>(10, m_wifiCompression-1);
        }
        
    }
}

}
