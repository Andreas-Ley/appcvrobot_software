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

#ifndef CONTROLSOCKET_H
#define CONTROLSOCKET_H

#include "config.h"
#include "Session.h"


#include <boost/asio.hpp>
#include <memory.h>

class ControlSocket
{
    public:
	
		std::string logmsges="";
        ControlSocket(boost::asio::io_context& ioContext);
        
        inline boost::asio::io_context& getIOContext() { return m_ioContext; }

        void onNewSession(const boost::system::error_code& error);
        void dropSession(Session *session);
		
    protected:
        boost::asio::io_context& m_ioContext;
        boostSockProt::acceptor m_acceptor;
        
        std::unique_ptr<Session> m_newSession;
        std::vector<std::unique_ptr<Session>> m_sessions;
        
        void startAccepting();
        
};


#endif // CONTROLSOCKET_H
