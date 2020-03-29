/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2020  <copyright holder> <email>
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

#ifndef SESSION_H
#define SESSION_H

#include "config.h"

#include <controlSocketProtocol/ControlSocketProtocol.h>

#include <boost/asio.hpp>

class ControlSocket;

class Session
{
    public:
        Session(ControlSocket &controlSocket);

        inline boostSockProt::socket &getSocket() { return m_socket; }
        inline void startSession() { startRecvRequestHead(); }
		bool LcdAq = false;
		bool MotorAq = false;
    protected:
        ControlSocket &m_controlSocket;
        boostSockProt::socket m_socket;
        
        robot::hardwareSocket::Request m_request;
        robot::hardwareSocket::Response m_response;
        
        void startRecvRequestHead();
        void onRequestHeadRecvd(const boost::system::error_code& error);

        void startRecvRequestBody(std::size_t bodySize);
        void onRequestBodyRecvd(const boost::system::error_code& error);

        void startSendResponse(std::size_t bodySize);
        void onResponseSent(const boost::system::error_code& error);
        
        void startSendFailureCode();
        void onResponseFailureCodeSent(const boost::system::error_code& error);
};


#endif // SESSION_H
