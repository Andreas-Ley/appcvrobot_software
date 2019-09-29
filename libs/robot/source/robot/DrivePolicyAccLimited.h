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

#ifndef DRIVEPOLICYACCLIMITED_H
#define DRIVEPOLICYACCLIMITED_H

#include "DrivePolicy.h"

#include <mutex>

/**
 * @todo write docs
 */
class DrivePolicyAccLimited : public DrivePolicy
{
    public:
        virtual ~DrivePolicyAccLimited() = default;
        
        virtual void setDesiredWheelSpeed(float left, float right);
        
        void getCurrentWheelSpeeds(float &left, float &right);
        void setMaxWheelAcceleration(float left, float right);

        virtual void fullStop() override;
        virtual void operate(float dt) override;
    protected:
        float m_desiredSpeedLeft = 0.0f;
        float m_desiredSpeedRight = 0.0f;

        float m_currentSpeedLeft = 0.0f;
        float m_currentSpeedRight = 0.0f;
        
        float m_maxAccelerationLeft = 2.0f;
        float m_maxAccelerationRight = 2.0f;
        
        std::mutex m_mutex;
};

#endif // DRIVEPOLICYACCLIMITED_H
