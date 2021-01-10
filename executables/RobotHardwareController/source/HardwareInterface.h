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

#ifndef HARDWAREINTERFACE_H
#define HARDWAREINTERFACE_H

#include <string>

#include <cstdint>

namespace hardwareInterface {
    
void init();
void shutdown();

namespace battery {

struct CellVoltages{
    float voltages[3];
};

CellVoltages getCellVoltages();
float getBatteryCurrentAmps();
    
}


unsigned getButtons();


namespace motors {

void enable(bool enable);
void setSpeed(float left, float right);
void getSteps(std::int16_t &left, std::int16_t &right);
float getControllerCPUUsage();

}

namespace lcd {

void initDisplay();
void clear();
void backlight(bool on);
void writeLine(const std::string &msg, unsigned line);
void setCursor(unsigned line, unsigned col);
void write(const std::string &msg);

}
    
}

#endif // HARDWAREINTERFACE_H
