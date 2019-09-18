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

#include "DrivePolicyEncoderPID.h"

#include <cmath>
#include <iostream>

void DrivePolicyEncoderPID::getCurrentWheelSpeeds(float &left, float &right)
{
    left = m_encoderFrequencyLeft.load() / MAX_TICKS_PER_SECOND;
    right = m_encoderFrequencyRight.load() / MAX_TICKS_PER_SECOND;
}

void DrivePolicyEncoderPID::fullStop()
{
    // todo: properly disable!
    m_desiredSpeedLeft.store(0.0f);
    m_desiredSpeedRight.store(0.0f);
}

void DrivePolicyEncoderPID::operate(float dt)
{
	DrivePolicy::operate(dt);
	
    float left, right;
    float d_left = m_desiredSpeedLeft.load();
    float d_right = m_desiredSpeedRight.load();
    m_pidLeft.operate(std::abs(d_left) * MAX_TICKS_PER_SECOND, m_encoderFrequencyLeft.load(), left, dt);
    m_pidRight.operate(std::abs(d_right) * MAX_TICKS_PER_SECOND, m_encoderFrequencyRight.load(), right, dt);
    left = std::max(left, 0.0f);
    right = std::max(right, 0.0f);
    left = std::copysign(left, d_left) / MAX_TICKS_PER_SECOND;
    right = std::copysign(right, d_right) / MAX_TICKS_PER_SECOND;
std::cout << "PID: " << left << " " << right << std::endl;
std::cout << "left desired actual: " << d_left << " " << m_encoderFrequencyLeft.load()/MAX_TICKS_PER_SECOND << std::endl;

	if (d_left == 0.0f) {
		m_pidLeft.fullStop();
		left = 0.0f;
	}
	if (d_right == 0.0f) {
		m_pidRight.fullStop();
		right = 0.0f;
	}

    outputDrive(left, right);
}

void DrivePolicyEncoderPID::PID::fullStop() 
{
	integrator = 0.0f;
	lastValue = 0.0f;
}


void DrivePolicyEncoderPID::PID::operate(float desired, float actual, float &control, float dt)
{
    float error = desired - actual;
    integrator += error;
    float diff = error - lastValue;
    lastValue = error;
    
    control = c_p * desired + c_i * integrator + c_d * diff;    
}
