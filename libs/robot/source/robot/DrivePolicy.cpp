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
#endif
}

void DrivePolicy::fullStop()
{
    outputDrive(0.0f, 0.0f);
}

namespace {

const unsigned calibrationCurveLeftMotor_inputOutputPairsSize = 6;
const std::pair<float, float> calibrationCurveLeftMotor_inputOutputPairs[calibrationCurveLeftMotor_inputOutputPairsSize] = {
    {0.0f, 0.0f},
    {0.5f, 0.42f},
    {0.6f, 0.54f},
    {0.7f, 0.58f},
    {0.8f, 0.68f},
    {1.0f, 0.985f},
};

float calibrate(float v) {
    float absv = std::abs(v);
    if (absv >= 1.0f)
        return std::copysign(1.0f, v);
    
    for (unsigned i = 1; i < calibrationCurveLeftMotor_inputOutputPairsSize; i++) {
        if (absv < calibrationCurveLeftMotor_inputOutputPairs[i].first) {
            float lambda = (absv - calibrationCurveLeftMotor_inputOutputPairs[i-1].first) / (calibrationCurveLeftMotor_inputOutputPairs[i].first - calibrationCurveLeftMotor_inputOutputPairs[i-1].first);
            float y = calibrationCurveLeftMotor_inputOutputPairs[i-1].second * (1.0f - lambda) + calibrationCurveLeftMotor_inputOutputPairs[i].second * lambda;
            return std::copysign(y, v);
        }
    }
    
    throw std::runtime_error("Reached unreachable code!");
}

}

void DrivePolicy::outputDrive(float left, float right)
{
    const float minPwm = 0.4f;

    if (left != 0.0f)
        left = std::copysign(minPwm, left) + left * (1.0f - minPwm);

    if (right != 0.0f)
        right = std::copysign(minPwm, right) + right * (1.0f - minPwm);
    
    left = calibrate(left);

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
