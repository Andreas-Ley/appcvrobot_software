#include <robot/Robot.h>
#include <robot/DrivePolicyEncoderPID.h>

#include <iostream>
#include <cstdio>

int main()
{
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(new DrivePolicyEncoderPID()));
    
#if 0
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
                left += 0.2f;
                std::cout << "Left " << left << std::endl;
                std::cout << "Right " << right << std::endl;
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(left, right);
            break;
            case 's':
                left -= 0.2f;
                std::cout << "Left " << left << std::endl;
                std::cout << "Right " << right << std::endl;
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(left, right);
            break;
            case 'r':
                right += 0.2f;
                std::cout << "Left " << left << std::endl;
                std::cout << "Right " << right << std::endl;
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(left, right);
            break;
            case 'f':
                right -= 0.2f;
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
    
#elif 1
    bool shutdown = false;
    while (!shutdown) {
        int c = std::getchar();
        switch (c) {
            case 'w':
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(1.0f, 1.0f);
                std::this_thread::sleep_for(std::chrono::milliseconds{5000});
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(0.0f, 0.0f);
            break;
            case 's':
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(-1.0f, -1.0f);
                std::this_thread::sleep_for(std::chrono::milliseconds{5000});
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(0.0f, 0.0f);
            break;
            case 'q':
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(0.8f, 1.0f);
                std::this_thread::sleep_for(std::chrono::milliseconds{5000});
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(0.0f, 0.0f);
            break;
            case 'e':
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(1.0f, 0.8f);
                std::this_thread::sleep_for(std::chrono::milliseconds{1500});
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(0.0f, 0.0f);
            break;
            case 'a':
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(-0.4f, 0.4f);
                std::this_thread::sleep_for(std::chrono::milliseconds{1500});
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(0.0f, 0.0f);
            break;
            case 'd':
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(0.4f, -0.4f);
                std::this_thread::sleep_for(std::chrono::milliseconds{1500});
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(0.0f, 0.0f);
            break;
            case ' ':
                shutdown = true;
            break;
        }
    }
#endif

    return 0;
}
