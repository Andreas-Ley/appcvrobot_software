#include <robot/Robot.h>
#include <robot/DrivePolicyEncoderPID.h>
#include <robot/WifiCommunication.h>
#include <robot/RemoteControlPolicy.h>
#include <robot/CameraSystem.h>


#include <iostream>
#include <cstdio>

int main()
{
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(new DrivePolicyEncoderPID()));
    robot::WifiCommunication *wifiCom;
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(wifiCom = new robot::WifiCommunication(1337)));
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(new robot::CameraSystem(wifiCom, 2'000'000))); // 2 MB/s
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(new robot::RemoteControlPolicy(wifiCom)));
    
    bool shutdown = false;
    while (!shutdown) {
        int c = std::getchar();
        switch (c) {
            case 'q':
                shutdown = true;
            break;
        }
    }

    return 0;
}
