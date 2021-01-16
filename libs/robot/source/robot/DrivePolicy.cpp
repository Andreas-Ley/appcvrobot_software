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

#include "Robot.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <stdexcept>

#include <iostream>

DrivePolicy::DrivePolicy()
{
    Robot::robot.getRobotHardware().acquireDrive();
}

DrivePolicy::~DrivePolicy()
{
    Robot::robot.getRobotHardware().releaseDrive();
}

void DrivePolicy::operate(float dt)
{

}


void DrivePolicy::fullStop()
{
    outputDrive(0.0f, 0.0f);
}

void DrivePolicy::outputDrive(float left, float right)
{
    Robot::robot.getRobotHardware().setDriveSpeed(left, right);
}
