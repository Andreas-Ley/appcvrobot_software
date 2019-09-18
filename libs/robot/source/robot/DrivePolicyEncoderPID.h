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

#ifndef DRIVEPOLICYENCODERPID_H
#define DRIVEPOLICYENCODERPID_H

#include "DrivePolicy.h"

#include <mutex>
#include <atomic>

/**
 * @todo write docs
 */
class DrivePolicyEncoderPID : public DrivePolicy
{
    public:
        const float MAX_TICKS_PER_SECOND = 40.0f; // 60 RPM
        
        virtual ~DrivePolicyEncoderPID() = default;
        
        virtual void setDesiredWheelSpeed(float left, float right) override { m_desiredSpeedLeft.store(left); m_desiredSpeedRight.store(right); }
        
        void getCurrentWheelSpeeds(float &left, float &right);
        virtual void fullStop() override;
        virtual void operate(float dt) override;
    protected:
        struct PID {
            float integrator = 0.0f;
            float lastValue = 0.0f;
            
            float c_p = 1.0f;
            float c_i = 0.1f;
            float c_d = 2.0f;
            
            void fullStop();
            void operate(float desired, float actual, float &control, float dt);
        };
        
        std::atomic<float> m_desiredSpeedLeft = std::atomic<float>(0.0f);
        std::atomic<float> m_desiredSpeedRight = std::atomic<float>(0.0f);

        PID m_pidLeft;
        PID m_pidRight;
};

#endif // DRIVEPOLICYENCODERPID_H
