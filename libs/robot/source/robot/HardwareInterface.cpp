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

#include "../../../../firmware/protocoll.h"

#include <mutex>
#include <chrono>
#include <thread>
#include <iostream>
#include <string.h>

namespace hardwareInterface {
    
enum {
    I2C_BUS = 1
};

std::mutex i2cBusMutex;

int i2cHandleController;
    
void init()
{
#ifndef BUILD_WITH_ROBOT_STUBS
    if (gpioInitialise() < 0)
        throw std::runtime_error("GPIO initialization failed!");


    bbI2COpen(2, 3, 100'000);
    i2cHandleController = 2;
//    i2cHandleController = i2cOpen(I2C_BUS, I2C_ADDRESS, 0);
#endif
}

void shutdown()
{
#ifndef BUILD_WITH_ROBOT_STUBS
    bbI2CClose(i2cHandleController);
//    i2cClose(i2cHandleController);

    gpioTerminate();
#endif
}

template<typename Type>
void readRegister(unsigned address, unsigned registerNumber, Type &data)
{
    std::lock_guard<std::mutex> lock(i2cBusMutex);

#ifndef BUILD_WITH_ROBOT_STUBS
    
    char command[] = {
        PI_I2C_ADDR,
        address,
        PI_I2C_START,
        PI_I2C_WRITE,
        1,
        registerNumber,
        PI_I2C_START,
        PI_I2C_READ,
        sizeof(data),
        PI_I2C_STOP,
        PI_I2C_END
    };

    int res;
    while ((res = bbI2CZip(i2cHandleController, command, sizeof(command), (char*)&data, sizeof(data))) < 0) {
        std::cout << "i2c write error: " << res << std::endl;
    }
#endif
}

template<typename Type>
void writeRegister(unsigned address, unsigned registerNumber, const Type &data)
{
    std::lock_guard<std::mutex> lock(i2cBusMutex);
#ifndef BUILD_WITH_ROBOT_STUBS
    
    char command[6+2+sizeof(data)] = {
        PI_I2C_ADDR,
        address,
        PI_I2C_START,
        PI_I2C_WRITE,
        1 + sizeof(data),
        registerNumber,
    };
    memcpy(command+6, &data, sizeof(data));
    command[6+sizeof(data)+0] = PI_I2C_STOP;
    command[6+sizeof(data)+1] = PI_I2C_END;

    int result;
    while ((result = bbI2CZip(i2cHandleController, command, sizeof(command), nullptr, 0)) == PI_I2C_WRITE_FAILED) {
        std::cout << "i2c write error: " << result << std::endl;
    }
    if (result != 0)
        throw std::runtime_error("i2c error!");
#endif
}

namespace battery {

float getCellVoltage(Cell cell)
{
    std::uint16_t buf;
    switch (cell) {
        case CELL_1:
            readRegister(I2C_ADDRESS, REGISTER_CELL_VOLTAGE_1, buf);
            return (buf / 1023.0f * 3.3) / 100 * (33+100);
        break;
        case CELL_2:
            return -1.0f;  // not working yet
        break;
        case CELL_3:
            return -1.0f;  // not working yet
        break;
    }
}

float getBatteryCurrentAmps()
{
    std::uint16_t buf;
    readRegister(I2C_ADDRESS, REGISTER_BATTERY_DRAW, buf);
    float vOut = (buf / 1023.0f * 3.3);
    
    float r1 = 10;
    float r2 = 33;
    float r3 = 33;
    float r4 = 22;

    //vOut = -v * r3/r1 + 
    
    float v = (5.0f * r4/(r2+r4)*(r1+r3)/r1 - vOut) * r1 / r3;

    //sensitivity: 185 mV/A
    return v / 0.185f;
}
    
}


namespace motors {

void enable(bool enable)
{
    writeRegister<char>(I2C_ADDRESS, REGISTER_ENABLE_MOTOR, enable?1:0);
}

void setSpeed(float left, float right)
{
    auto speed2delay = [](float speed)->std::int16_t{
        speed = std::min(std::max(speed, -1.0f), 1.0f);
        
        const unsigned minDelay = 10; // 5kHz / 25 / 200 steps/rot = 1 rot/second
        
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
    
    writeRegister(I2C_ADDRESS, REGISTER_SET_TARGET_SPEED, delays);
}

void getSteps(std::int16_t &left, std::int16_t &right)
{
    std::int16_t buf[2] = {};
    readRegister(I2C_ADDRESS, REGISTER_STEPS_MOVED, buf);
    left = buf[0];
    right = buf[1];
}

float getControllerCPUUsage()
{
    std::lock_guard<std::mutex> lock(i2cBusMutex);

    char buf;
    readRegister(I2C_ADDRESS, REGISTER_CPU_USAGE, buf);
    return buf / 255.0f;
}

}

namespace lcd {

enum {
    LCD_ADDRESS = 0x27,

    LCD_CLEARDISPLAY = 0x01,
    LCD_RETURNHOME = 0x02,
    LCD_ENTRYMODESET = 0x04,
    LCD_DISPLAYCONTROL = 0x08,
    LCD_CURSORSHIFT = 0x10,
    LCD_FUNCTIONSET = 0x20,
    LCD_SETCGRAMADDR = 0x40,
    LCD_SETDDRAMADDR = 0x80,

    LCD_ENTRYRIGHT = 0x00,
    LCD_ENTRYLEFT = 0x02,
    LCD_ENTRYSHIFTINCREMENT = 0x01,
    LCD_ENTRYSHIFTDECREMENT = 0x00,

    LCD_DISPLAYON = 0x04,
    LCD_DISPLAYOFF = 0x00,
    LCD_CURSORON = 0x02,
    LCD_CURSOROFF = 0x00,
    LCD_BLINKON = 0x01,
    LCD_BLINKOFF = 0x00,

    LCD_DISPLAYMOVE = 0x08,
    LCD_CURSORMOVE = 0x00,
    LCD_MOVERIGHT = 0x04,
    LCD_MOVELEFT = 0x00,

    LCD_8BITMODE = 0x10,
    LCD_4BITMODE = 0x00,
    LCD_2LINE = 0x08,
    LCD_1LINE = 0x00,
    LCD_5x10DOTS = 0x04,
    LCD_5x8DOTS = 0x00,

    LCD_BACKLIGHT = 0x08,
    LCD_NOBACKLIGHT = 0x00,

    En = 0b00000100,
    Rw = 0b00000010,
    Rs = 0b00000001,

};


class IOExpander {
    public:
        std::uint8_t data = 0;
        bool rs = false;
        bool rw = false;
        bool en = false;
        bool backlight = false;
        IOExpander(unsigned address) : address(address) {
        }

        void update() {
            #ifndef BUILD_WITH_ROBOT_STUBS

            std::uint8_t packet = 
                        (rs?Rs:0) |
                        (rw?Rw:0) |
                        (en?En:0) |
                        (backlight?LCD_BACKLIGHT:0) |
                        (data << 4);

            char command[] = {
                PI_I2C_ADDR,
                address,
                PI_I2C_START,
                PI_I2C_WRITE,
                1,
                packet,
                PI_I2C_STOP,
                PI_I2C_END
            };

            int result;
            while ((result = bbI2CZip(i2cHandleController, command, sizeof(command), nullptr, 0)) == PI_I2C_WRITE_FAILED) {
                std::cout << "i2c write error: " << result << std::endl;
            }
            if (result != 0)
                throw std::runtime_error("i2c error!");
                
            #endif            
        }

        void send4bits(std::uint8_t bits) {
            data = bits;
            {
                std::lock_guard<std::mutex> lock(i2cBusMutex);

                en = false;
                update();
                en = true;
                update();
            }
            std::this_thread::sleep_for(std::chrono::microseconds{500});
            {
                std::lock_guard<std::mutex> lock(i2cBusMutex);
                en = false;
                update();
            }
            std::this_thread::sleep_for(std::chrono::microseconds{100});
        }
        void send8bits(std::uint8_t bits) {
            send4bits((bits >> 4) & 0x0F);
            send4bits(bits & 0x0F);
        }
    protected:
        unsigned address;
};


IOExpander expander(LCD_ADDRESS);

void initDisplay()
{
    expander.backlight = true;


    expander.rs = false;
    expander.rw = false;

    // Init by Instruction (forces reset), p45
    expander.send4bits(3);
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
    expander.send4bits(3);
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
    expander.send4bits(3);
    std::this_thread::sleep_for(std::chrono::milliseconds{5});


    // Init sequence 4bit, p42
    expander.send4bits(LCD_FUNCTIONSET >> 4);

    expander.send8bits(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS | LCD_4BITMODE);
    expander.send8bits(LCD_DISPLAYCONTROL | LCD_DISPLAYON);

    expander.send8bits(LCD_CLEARDISPLAY);
    expander.send8bits(LCD_ENTRYMODESET | LCD_ENTRYLEFT);

    std::this_thread::sleep_for(std::chrono::milliseconds{200});
}

void clear()
{
    expander.send8bits(LCD_CLEARDISPLAY);
    expander.send8bits(LCD_RETURNHOME);
}

void backlight(bool on)
{
    std::lock_guard<std::mutex> lock(i2cBusMutex);
    expander.backlight = on;
    expander.en = false;
    expander.update();
}

void writeLine(const std::string &msg, unsigned line)
{
    setCursor(line, 0);
    write(msg);
}

void setCursor(unsigned line, unsigned col)
{
    expander.rs = false;
    expander.send8bits(LCD_SETDDRAMADDR | line * 0x40 + col);
}

void write(const std::string &msg)
{
    expander.rs = true;
    for (auto c : msg)
        expander.send8bits(c);
}


}
    
}



