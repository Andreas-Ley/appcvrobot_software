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

#include "DrivePolicy.h"

#ifndef BUILD_WITH_ROBOT_STUBS
#include <pigpio.h>
#endif

#include <algorithm>
#include <cmath>
#include <utility>
#include <stdexcept>

#include <iostream>

void wheelEncoderTriggerCallback(int event, int level, std::uint32_t tick, void *userdata)
{
    ((DrivePolicy*)userdata)->wheelEncoderTrigger(event, level, tick);
}

DrivePolicy::DrivePolicy()
{
#ifndef BUILD_WITH_ROBOT_STUBS
    gpioSetMode(GPIO_PIN_PWM_LEFT, PI_OUTPUT);
    gpioSetMode(GPIO_PIN_DIRECTION_A_LEFT, PI_OUTPUT);
    gpioSetMode(GPIO_PIN_DIRECTION_B_LEFT, PI_OUTPUT);
    gpioSetMode(GPIO_PIN_PWM_RIGHT, PI_OUTPUT);
    gpioSetMode(GPIO_PIN_DIRECTION_A_RIGHT, PI_OUTPUT);
    gpioSetMode(GPIO_PIN_DIRECTION_B_RIGHT, PI_OUTPUT);
    
    gpioSetPWMfrequency(GPIO_PIN_PWM_LEFT, 2'000); 
    gpioSetPWMfrequency(GPIO_PIN_PWM_RIGHT, 2'000); 

    gpioSetMode(GPIO_PIN_ENCODER_LEFT, PI_INPUT);
    gpioSetMode(GPIO_PIN_ENCODER_RIGHT, PI_INPUT);
    
    gpioSetAlertFuncEx(GPIO_PIN_ENCODER_LEFT, &wheelEncoderTriggerCallback, this);
    gpioSetAlertFuncEx(GPIO_PIN_ENCODER_RIGHT, &wheelEncoderTriggerCallback, this);
#endif
}


void DrivePolicy::wheelEncoderTrigger(int event, int level, uint32_t tick)
{
    if ((level == 0) || (level == 1)) { // falling or rising edge
        if (event == GPIO_PIN_ENCODER_LEFT) {
            m_encoderTriggerTimeLeft.store(tick - m_lastEncoderTriggerLeft);
            m_lastEncoderTriggerLeft = tick;
            m_encoderTriggerLeft++;
        }
        if (event == GPIO_PIN_ENCODER_RIGHT) {
            m_encoderTriggerTimeRight.store(tick - m_lastEncoderTriggerRight);
            m_lastEncoderTriggerRight = tick;
            m_encoderTriggerRight++;
        }
    }
}

void DrivePolicy::operate(float dt)
{
    unsigned ticksLeft = m_encoderTriggerLeft.exchange(0);
    if (ticksLeft == 0) {
        m_itersWithNoTicksLeft++;
    } else
        m_itersWithNoTicksLeft = 0;
    
    if (m_itersWithNoTicksLeft > 20)
        m_encoderFrequencyLeft.store(0.0f);
    else
        m_encoderFrequencyLeft.store(m_encoderFrequencyLeft.load() * 0.5f + 0.5f * 1e6f / m_encoderTriggerTimeLeft.load());
    
    unsigned ticksRight = m_encoderTriggerRight.exchange(0);
    if (ticksRight == 0) {
        m_itersWithNoTicksRight++;
    } else
        m_itersWithNoTicksRight = 0;
    
    if (m_itersWithNoTicksRight > 20)
        m_encoderFrequencyRight.store(0.0f);
    else
        m_encoderFrequencyRight.store(m_encoderFrequencyRight.load() * 0.5f + 0.5f * 1e6f / m_encoderTriggerTimeRight.load());
    
    
    //std::cout << m_encoderFrequencyLeft.load() << "  " << m_encoderFrequencyRight.load() << std::endl;
}


void DrivePolicy::fullStop()
{
    outputDrive(0.0f, 0.0f);
}

void DrivePolicy::outputDrive(float left, float right)
{
	float minimum = 0.3f;
	if (left != 0.0f)
		left = std::copysign(std::max(0.0f, std::abs(left-minimum)/(1.0f - minimum))+minimum, left);
	if (right != 0.0f)
		right = std::copysign(std::max(0.0f, std::abs(right-minimum)/(1.0f - minimum))+minimum, right);
	
#ifndef BUILD_WITH_ROBOT_STUBS
    if (left > 0.0f) {
        gpioWrite(GPIO_PIN_DIRECTION_A_LEFT, 1);
        gpioWrite(GPIO_PIN_DIRECTION_B_LEFT, 0);
        
        gpioPWM(GPIO_PIN_PWM_LEFT, std::min<int>(std::abs(left) * 255.0f, 255));
    } else if (left < 0.0f) {
        gpioWrite(GPIO_PIN_DIRECTION_A_LEFT, 0);
        gpioWrite(GPIO_PIN_DIRECTION_B_LEFT, 1);

        gpioPWM(GPIO_PIN_PWM_LEFT, std::min<int>(std::abs(left) * 255.0f, 255));
    } else {
        gpioWrite(GPIO_PIN_DIRECTION_A_LEFT, 0);
        gpioWrite(GPIO_PIN_DIRECTION_B_LEFT, 0);

        gpioPWM(GPIO_PIN_PWM_LEFT, 0);
    }
    
    if (right > 0.0f) {
        gpioWrite(GPIO_PIN_DIRECTION_A_RIGHT, 1);
        gpioWrite(GPIO_PIN_DIRECTION_B_RIGHT, 0);
        
        gpioPWM(GPIO_PIN_PWM_RIGHT, std::min<int>(std::abs(right) * 255.0f, 255));
    } else if (right < 0.0f) {
        gpioWrite(GPIO_PIN_DIRECTION_A_RIGHT, 0);
        gpioWrite(GPIO_PIN_DIRECTION_B_RIGHT, 1);

        gpioPWM(GPIO_PIN_PWM_RIGHT, std::min<int>(std::abs(right) * 255.0f, 255));
    } else {
        gpioWrite(GPIO_PIN_DIRECTION_A_RIGHT, 0);
        gpioWrite(GPIO_PIN_DIRECTION_B_RIGHT, 0);

        gpioPWM(GPIO_PIN_PWM_RIGHT, 0);
   }    
#endif
}
