#include <robot/Robot.h>
#include <robot/DrivePolicyAccLimited.h>

#include <iostream>
#include <cstdio>

int main()
{
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(new DrivePolicyAccLimited()));
    
    float left = 0.0f;
    float right = 0.0f;
    
    bool shutdown = false;
    while (!shutdown) {
        int c = std::getchar();
        switch (c) {
            case 'q':
                shutdown = true;
            break;
            case 'w':
                left += 0.1f;
                std::cout << "Left " << left << std::endl;
                std::cout << "Right " << right << std::endl;
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(left, right);
            break;
            case 's':
                left -= 0.1f;
                std::cout << "Left " << left << std::endl;
                std::cout << "Right " << right << std::endl;
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(left, right);
            break;
            case 'r':
                right += 0.1f;
                std::cout << "Left " << left << std::endl;
                std::cout << "Right " << right << std::endl;
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(left, right);
            break;
            case 'f':
                right -= 0.1f;
                std::cout << "Left " << left << std::endl;
                std::cout << "Right " << right << std::endl;
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(left, right);
            break;
            case ' ':
                left = 0.0f;
                right = 0.0f;
                std::cout << "Left " << left << std::endl;
                std::cout << "Right " << right << std::endl;
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(left, right);
            break;
        }
    }
    
    return 0;
}
