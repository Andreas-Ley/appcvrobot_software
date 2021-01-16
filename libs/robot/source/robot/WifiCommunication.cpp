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

#include "WifiCommunication.h"

#include "Robot.h"

#include <boost/bind.hpp>

#include <iostream>
#include <limits>

namespace robot {
 
using boost::asio::ip::udp;

    
WifiCommunication::WifiCommunication(unsigned udpPort) :
        m_udpSocket(Robot::robot.getIoService(), udp::endpoint(udp::v4(), udpPort))
{
    startReceivingPacket();
}

WifiCommunication::~WifiCommunication()
{
    {
        std::lock_guard<std::mutex> lock(m_sockMutex);
        m_udpSocket.cancel();
        m_udpSocket.close();
    }
}

void WifiCommunication::send(Packet packet, bool highPrio)
{
    if (highPrio)
        enqueueTxPacket(std::move(packet), m_highPrioTx);
    else
        enqueueTxPacket(std::move(packet), m_lowPrioTx);
}

void WifiCommunication::startReceivingPacket()
{
    m_udpSocket.async_receive_from(boost::asio::buffer(m_receiveBuffer), m_lastRecievedEndpoint,
        boost::bind(&WifiCommunication::onReceivedPacket, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
}

void WifiCommunication::onReceivedPacket(const boost::system::error_code &error, std::size_t size)
{
    if (!error) {
        bool connected = false;
        for (auto &c : m_connections) 
            if (c.senderEndpoint == m_lastRecievedEndpoint)
                connected = true;
            
        if (connected) {
            
            if (size < sizeof(MessageHeader))
                std::cout << "Dropping broken packet" << std::endl;
            else {
                const MessageHeader *header = (const MessageHeader*) m_receiveBuffer.data();
                
                switch (header->messageType) {
                    case MESSAGE_ID_REMOTE_STEER_CMD: {
                        const RemoteSteerCmdPacket *steerPacket = (const RemoteSteerCmdPacket*) m_receiveBuffer.data();
                        int diff = (int)steerPacket->header.sequenceNumber - (int)m_lastRemoteSteerCommand.sequenceNumber;
                        if (diff < 0) break;
                        
                        std::lock_guard<std::mutex> lock(m_steerCommandMutex);
                        m_lastRemoteSteerCommand.left = steerPacket->left / (float) std::numeric_limits<decltype(steerPacket->left)>::max();
                        m_lastRemoteSteerCommand.right = steerPacket->right / (float) std::numeric_limits<decltype(steerPacket->right)>::max();
                        m_lastRemoteSteerCommand.sequenceNumber = steerPacket->header.sequenceNumber;
                        m_lastRemoteSteerCommand.whenRecieved = std::chrono::steady_clock::now();
                    } break;
                    default:
                    break;
                }
            }
            
        } else {
            std::string msg(m_receiveBuffer.data(), m_receiveBuffer.data()+size);
            if (msg == "I, for one, welcome our new machine overlords") {
                std::scoped_lock<std::mutex> lock(m_TxMutex);
                m_connections.push_back({m_lastRecievedEndpoint});
            } else {
                std::string returnMessage = "Speak the passphrase to connect";
                
                Packet packet;
                packet.data = std::vector<char>(returnMessage.begin(), returnMessage.end());
                packet.destination = m_lastRecievedEndpoint;
                
                enqueueTxPacket(std::move(packet), m_highPrioTx);
            }
        }
        
        startReceivingPacket();
    } else {
        std::cout << "Error occured while waiting for next udp packet: " << error << std::endl;
    }
}

void WifiCommunication::enqueueTxPacket(Packet packet, TxChannel &channel)
{
    std::scoped_lock<std::mutex> lock(m_TxMutex);
    if (!m_sendingTxPacket) {
        m_txPacket = std::move(packet);
        m_broadcast2ConnectionsNextIndex = 0;
        if (!m_txPacket.broadcast2Connections || !m_connections.empty())
            sendTxPacket();
        else {
        }
    } else {
        channel.queuedBytes += packet.data.size();
        channel.queue.push_back(std::move(packet));
    }
}

void WifiCommunication::sendTxPacket()
{
    if (m_txPacket.broadcast2Connections) {
        m_sendingTxPacket = true;
        m_udpSocket.async_send_to(
            boost::asio::buffer(m_txPacket.data), m_connections[m_broadcast2ConnectionsNextIndex++].senderEndpoint,
            boost::bind(&WifiCommunication::onSendTxPacket, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
    } else {
        m_sendingTxPacket = true;
        m_udpSocket.async_send_to(
            boost::asio::buffer(m_txPacket.data), m_txPacket.destination,
            boost::bind(&WifiCommunication::onSendTxPacket, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
    }
}

void WifiCommunication::onSendTxPacket(const boost::system::error_code &error, std::size_t size)
{
    std::scoped_lock<std::mutex> lock(m_TxMutex);
    m_sendingTxPacket = false;
    
    if (m_txPacket.broadcast2Connections && (m_broadcast2ConnectionsNextIndex < m_connections.size())) {
        sendTxPacket();
        return;
    }
    
    if (!m_highPrioTx.queue.empty()) {
        m_txPacket = std::move(m_highPrioTx.queue.front());
        m_highPrioTx.queue.pop_front();
        m_broadcast2ConnectionsNextIndex = 0;
        m_highPrioTx.queuedBytes -= m_txPacket.data.size();


        sendTxPacket();
        return;
    }
    if (!m_lowPrioTx.queue.empty()) {
        m_txPacket = std::move(m_lowPrioTx.queue.front());
        m_lowPrioTx.queue.pop_front();
        m_broadcast2ConnectionsNextIndex = 0;
        m_lowPrioTx.queuedBytes -= m_txPacket.data.size();

        sendTxPacket();
        return;
    }
}

}
