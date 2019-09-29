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

#include "Robot.h"

#include "HardwareInterface.h"

#ifndef BUILD_WITH_ROBOT_STUBS
#include <pigpio.h>
#endif

#include <chrono>
#include <stdexcept>
#include <functional>


Robot Robot::robot;


Robot::Robot()
{
#ifndef BUILD_WITH_ROBOT_STUBS
    if (gpioInitialise() < 0)
        throw std::runtime_error("GPIO initialization failed!");
#endif
    hardwareInterface::init();
    
    m_terminate.store(false);
    m_thread = std::thread(std::bind(&Robot::threadBody, this));
    m_threadSlow = std::thread(std::bind(&Robot::threadBodySlow, this));
}

Robot::~Robot()
{
    m_terminate.store(true);
    m_thread.join();
    m_threadSlow.join();
    
    for (auto &s : m_subsystems)
        s->fullStop();
    
    hardwareInterface::shutdown();
#ifndef BUILD_WITH_ROBOT_STUBS
    gpioTerminate();
#endif
}

void Robot::addSubsystem(std::unique_ptr<Subsystem> subsystem)
{
    DrivePolicy* dp = dynamic_cast<DrivePolicy*>(subsystem.get());
    if (dp != nullptr) {
        if (m_drivePolicy != nullptr)
            throw std::runtime_error("Robot already has a drive policy!");
        
        m_drivePolicy = dp;
    }
    // todo: make thread safe!!
    m_subsystems.push_back(std::move(subsystem));
}

void Robot::threadBody()
{
    auto lastStart = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(m_threadTimeStep);
    
    while (!m_terminate.load()) {
        auto iterStart = std::chrono::high_resolution_clock::now();
        auto deltaT = iterStart - lastStart;
        lastStart = iterStart;
        
        float dt = std::chrono::duration_cast<std::chrono::microseconds>(deltaT).count() * 1e-6f;
        
        for (auto &s : m_subsystems)
            s->operate(dt);
        
        auto iterEnd = std::chrono::high_resolution_clock::now();
        auto elapsed = iterEnd - iterStart;
        if (elapsed < m_threadTimeStep) {
            std::this_thread::sleep_for(m_threadTimeStep - elapsed);
        }
    }
}

void Robot::threadBodySlow()
{
    auto lastStart = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(m_threadTimeStepSlow);
    
    while (!m_terminate.load()) {
        auto iterStart = std::chrono::high_resolution_clock::now();
        auto deltaT = iterStart - lastStart;
        lastStart = iterStart;
        
        float dt = std::chrono::duration_cast<std::chrono::microseconds>(deltaT).count() * 1e-6f;
        
        for (auto &s : m_subsystems)
            s->operateSlow(dt);
        
        auto iterEnd = std::chrono::high_resolution_clock::now();
        auto elapsed = iterEnd - iterStart;
        if (elapsed < m_threadTimeStepSlow) {
            std::this_thread::sleep_for(m_threadTimeStepSlow - elapsed);
        }
    }
}
