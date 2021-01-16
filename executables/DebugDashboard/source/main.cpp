#include <robot/Robot.h>
#include <robot/DrivePolicyAccLimited.h>
#include <robot/WifiCommunication.h>
#include <robot/RemoteControlPolicy.h>
#include <robot/CameraSystem.h>
#include <robot/SystemMonitoring.h>
#ifdef BUILD_WITH_ORB_SLAM
#include <robot/SLAMSystem.h>
#endif


#include <iostream>
#include <cstdio>

int main()
{
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(new DrivePolicyAccLimited()));
    robot::WifiCommunication *wifiCom;
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(wifiCom = new robot::WifiCommunication(1337)));
    robot::CameraSystem *cam;
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(cam = new robot::CameraSystem(wifiCom, 2'000'000))); // 2 MB/s
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(new robot::RemoteControlPolicy(wifiCom)));
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(new robot::SystemMonitoring(wifiCom)));
#ifdef BUILD_WITH_ORB_SLAM
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(new robot::SLAMSystem(*cam, wifiCom, 100'000))); // 100KB/s
#endif

    Robot::robot.startIoService();
    
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
