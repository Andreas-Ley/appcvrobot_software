#include <robot/Robot.h>
#include <robot/DrivePolicyAccLimited.h>

#include <System.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <iostream>
#include <cstdio>

int main()
{
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(new DrivePolicyAccLimited()));
 
    cv::VideoCapture videoCapture;

    if (!videoCapture.open(0)) {
        std::cerr << "Error opening video" << std::endl;
        return 0;
    }
    
    ORB_SLAM2::System SLAM("Vocabulary/ORBvoc.txt", "../RPiCameraWide.yaml", ORB_SLAM2::System::MONOCULAR, true);    
 
    auto start = std::chrono::steady_clock::now();
    cv::Mat frame;
    while (true) {
        videoCapture >> frame;
        
        auto t = std::chrono::steady_clock::now();
        
        double timestamp = std::chrono::duration_cast<std::chrono::duration<double> >(t - start).count();
        
        if( frame.empty() ) break; // end of video stream
        
        cv::Mat pose = SLAM.TrackMonocular(frame, timestamp);
        std::cout << "Pose: " << pose << std::endl;
    }
    
    SLAM.Shutdown();

    
    return 0;
}
