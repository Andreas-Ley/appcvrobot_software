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

#include "HardwareInterface.h"

#ifndef BUILD_WITH_ROBOT_STUBS
#include <pigpio.h>
#endif

#include "../../../firmware/protocoll.h"

#include <mutex>


namespace hardwareInterface {
    
enum {
    I2C_BUS = 1
};

std::mutex i2cBusMutex;

int i2cHandleController;
    
void init()
{
#ifndef BUILD_WITH_ROBOT_STUBS
    i2cHandleController = i2cOpen(I2C_BUS, I2C_ADDRESS, 0);
#endif
}

void shutdown()
{
#ifndef BUILD_WITH_ROBOT_STUBS
    i2cClose(i2cHandleController);
#endif
}

namespace motors {

void enable(bool enable)
{
    std::lock_guard<std::mutex> lock(i2cBusMutex);
#ifndef BUILD_WITH_ROBOT_STUBS
    if (i2cWriteByteData(i2cHandleController, REGISTER_ENABLE_MOTOR, enable?1:0) != 0)
        throw std::runtime_error("i2c error!");
#endif
}

void setSpeed(float left, float right)
{
    std::lock_guard<std::mutex> lock(i2cBusMutex);

    auto speed2delay = [](float speed)->std::int16_t{
        speed = std::min(std::max(speed, -1.0f), 1.0f);
        
        const unsigned minDelay = 25; // 5kHz / 25 / 200 steps/rot = 1 rot/second
        
        const unsigned maxDelay = 1000;
        const float minSpeed = minDelay / (float) maxDelay;
        
        if (std::abs(speed) < minSpeed)
            return 0;
        
        int delay = (int) minDelay / std::abs(speed);
        if (speed > 0.0f)
            return delay;
        else
            return -delay;
    };
    
    std::int16_t delays[2] = {
        speed2delay(left),
        speed2delay(right)
    };
    
#ifndef BUILD_WITH_ROBOT_STUBS
    if (i2cWriteBlockData(i2cHandleController, REGISTER_SET_TARGET_SPEED, (char*)delays, 4) != 0)
        throw std::runtime_error("i2c error!");
#endif
    
}

void getSteps(std::int16_t &left, std::int16_t &right)
{
    std::lock_guard<std::mutex> lock(i2cBusMutex);

    std::int16_t buf[32/2] = {};
#ifndef BUILD_WITH_ROBOT_STUBS
    if (i2cReadBlockData(i2cHandleController, REGISTER_STEPS_MOVED, (char*)buf) != 0)
        throw std::runtime_error("i2c error!");
#endif
    left = buf[0];
    right = buf[1];
}

float getControllerCPUUsage()
{
    std::lock_guard<std::mutex> lock(i2cBusMutex);

    int result = 0;
    
#ifndef BUILD_WITH_ROBOT_STUBS
    if ((result = i2cReadByteData(i2cHandleController, REGISTER_CPU_USAGE)) < 0)
        throw std::runtime_error("i2c error!");
#endif
    return result / 255.0f;
}

}

//void writeLCD(const std::string &msg);
    
}


