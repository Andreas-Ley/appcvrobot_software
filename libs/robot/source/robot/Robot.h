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

#ifndef ROBOT_H
#define ROBOT_H

#include "Subsystem.h"
#include "DrivePolicy.h"

#include <memory>
#include <list>
#include <atomic>
#include <thread>

/**
 * @todo write docs
 */
class Robot
{
    public:
        void fullStop();
        
        static Robot robot;
        
        void addSubsystem(std::unique_ptr<Subsystem> subsystem);
        
        
        inline DrivePolicy* getDrivePolicy() { return m_drivePolicy; }
    protected:
        std::chrono::duration<long int, std::micro> m_threadTimeStep = std::chrono::duration<long int, std::micro>(10'000);
        std::chrono::duration<long int, std::micro> m_threadTimeStepSlow = std::chrono::duration<long int, std::micro>(100'000);
        
        DrivePolicy* m_drivePolicy = nullptr;
        std::list<std::unique_ptr<Subsystem>> m_subsystems;
        
        std::atomic_bool m_terminate;
        std::thread m_thread;
        std::thread m_threadSlow;
        
        void threadBody();
        void threadBodySlow();


        Robot();
        ~Robot();
};


#endif // ROBOT_H
