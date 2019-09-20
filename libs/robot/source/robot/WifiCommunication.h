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

#ifndef WIFICOMMUNICATION_H
#define WIFICOMMUNICATION_H

#include "Subsystem.h"

#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <atomic>
#include <mutex>
#include <thread>
#include <list>
#include <vector>


namespace robot {
    
/**
 * @todo write docs
 */
class WifiCommunication : public Subsystem
{
    public:
        enum {
            MAX_PACKET_SIZE = 1024,
            MESSAGE_ID_CAMERA_TILE = 0x10,
            MESSAGE_ID_SLAM_POSE = 0x20,
            MESSAGE_ID_SLAM_MAP_SLICE = 0x21,
        };
        
        struct MessageHeader {
            std::size_t sequenceNumber;
        };
        
        struct Packet {
            boost::asio::ip::udp::endpoint destination;
            bool broadcast2Connections = false;
            std::vector<char> data;
        };
        
        WifiCommunication(unsigned udpPort);
        virtual ~WifiCommunication();
        
        unsigned getTxQueueLength() { return m_highPrioTx.queuedBytes.load() + m_lowPrioTx.queuedBytes.load(); }
        void send(Packet packet, bool highPrio);
    protected:
        std::atomic<bool> m_shutdown = std::atomic<bool>(false);
        std::mutex m_sockMutex;
        std::thread m_thread;
        boost::asio::io_service m_ioService;
        
        boost::asio::ip::udp::socket m_udpSocket;
        boost::asio::ip::udp::endpoint m_lastRecievedEndpoint;
        boost::array<char, MAX_PACKET_SIZE> m_receiveBuffer;
        
        void startReceivingPacket();
        void onReceivedPacket(const boost::system::error_code &error, std::size_t size);
        
        struct TxChannel {
            std::list<Packet> queue;
            
            std::atomic<std::uint32_t> queuedBytes;
        };
        
        std::mutex m_TxMutex;
        Packet m_txPacket;
        bool m_sendingTxPacket = false;
        unsigned m_broadcast2ConnectionsNextIndex = 0;
        TxChannel m_highPrioTx;
        TxChannel m_lowPrioTx;
        
        struct Connections {
            boost::asio::ip::udp::endpoint senderEndpoint;
            //std::size_t nextSequenceNumber = 0;
        };
        
        std::vector<Connections> m_connections;
        
        void enqueueTxPacket(Packet packet, TxChannel &channel);
        void sendTxPacket();
        void onSendTxPacket(const boost::system::error_code &error, std::size_t size);
        
};

}

#endif // WIFICOMMUNICATION_H
