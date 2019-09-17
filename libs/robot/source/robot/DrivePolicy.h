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

#ifndef DRIVEPOLICY_H
#define DRIVEPOLICY_H

#include "Subsystem.h"

/**
 * @todo write docs
 */
class DrivePolicy : public Subsystem
{
    public:
        enum {
            GPIO_PIN_PWM_LEFT = 12,//18,
            GPIO_PIN_DIRECTION_A_LEFT = 6,//17,
            GPIO_PIN_DIRECTION_B_LEFT = 19,//22,
            GPIO_PIN_PWM_RIGHT = 13,//27,
            GPIO_PIN_DIRECTION_A_RIGHT = 16,//23,
            GPIO_PIN_DIRECTION_B_RIGHT = 20,//24,
        };
        
        DrivePolicy();
        virtual ~DrivePolicy() = default;
        
        virtual void fullStop() override;
        virtual void setDesiredWheelSpeed(float left, float right) = 0;
    protected:
        
        void outputDrive(float left, float right);
};

#endif // DRIVEPOLICY_H
