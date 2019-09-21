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

#include "RemoteControlPolicy.h"
#include "Robot.h"

#include <chrono>
#include <iostream>

namespace robot {

RemoteControlPolicy::RemoteControlPolicy(WifiCommunication *wifiCommunication) : m_wifiCommunication(wifiCommunication) {
}

void RemoteControlPolicy::operate(float dt) { 
    
    auto lastCmd = m_wifiCommunication->getLastSteerCommand();
    
    auto age = std::chrono::steady_clock::now() - lastCmd.whenRecieved;
    if (age > std::chrono::milliseconds(1000)) {
		std::cout << "No remote control signal, stopping" << std::endl;
        Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(0.0f, 0.0f);
    } else
        Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(lastCmd.left, lastCmd.right);
    
}
    
    
}
