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

#include "ControlSocket.h"
#include "Logger.h"

#include <boost/bind.hpp>




ControlSocket::ControlSocket(boost::asio::io_context& ioContext, Manager& manager) : 
    m_ioContext(ioContext), m_manager(manager), m_acceptor(ioContext, boostSockProt::endpoint(socketFile))
{
    startAccepting();
}



void ControlSocket::onNewSession(const boost::system::error_code& error)
{
    if (!error) {
        m_newSession->startSession();
        m_sessions.push_back(std::move(m_newSession));
    } else {
        logger.log(0) << "Cannot start Session"  << std::endl;
    }

    startAccepting();  
}

void ControlSocket::dropSession(Session *session)
{
    for (auto &s : m_sessions)
        if (s.get() == session) {
            s = std::move(m_sessions.back());
            m_sessions.pop_back();
            return;
        }
        
    throw std::runtime_error("Invalid session to drop!");
}


void ControlSocket::startAccepting()
{
    m_newSession.reset(new Session(*this, m_manager));
    m_acceptor.async_accept(m_newSession->getSocket(),
        boost::bind(&ControlSocket::onNewSession, this, boost::asio::placeholders::error));
}

