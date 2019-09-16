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

#include "DrivePolicyAccLimited.h"

#include <cmath>

void DrivePolicyAccLimited::setDesiredWheelSpeed(float left, float right)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_desiredSpeedLeft = left;
    m_desiredSpeedRight = right;
}

void DrivePolicyAccLimited::getCurrentWheelSpeeds(float &left, float &right)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    left = m_currentSpeedLeft;
    right = m_currentSpeedRight;
}

void DrivePolicyAccLimited::setMaxWheelAcceleration(float left, float right)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_maxAccelerationLeft = left;
    m_maxAccelerationRight = right;
    
}

void DrivePolicyAccLimited::fullStop()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_desiredSpeedLeft = 0.0f;
    m_desiredSpeedRight = 0.0f;

    m_currentSpeedLeft = 0.0f;
    m_currentSpeedRight = 0.0f;
    outputDrive(m_currentSpeedLeft, m_currentSpeedRight);
}


void DrivePolicyAccLimited::operate(float dt)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    auto update = [](float &current, float desired, float maxChange) {
        float diff = desired - current;
        if (std::abs(diff) < maxChange)
            current = desired;
        else 
            current += std::copysign(maxChange, diff);
    };
    
    update(
        m_currentSpeedLeft,
        m_desiredSpeedLeft,
        m_maxAccelerationLeft * dt
    );
    update(
        m_currentSpeedRight,
        m_desiredSpeedRight,
        m_maxAccelerationRight * dt
    );
    outputDrive(m_currentSpeedLeft, m_currentSpeedRight);
}
