#include <robot/Robot.h>
#include <robot/DrivePolicyAccLimited.h>

#include <System.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <Eigen/Dense>

#include <chrono>
#include <iostream>
#include <cstdio>

class CameraSystem : public Subsystem
{
    public:
        CameraSystem() {
            system("v4l2-ctl -c iso_sensitivity_auto=0");
            system("v4l2-ctl -c iso_sensitivity=4");
            system("v4l2-ctl -c scene_mode=11");
            system("v4l2-ctl -c video_bitrate=25000000");
            system("v4l2-ctl --set-fmt-video=width=640,height=480,pixelformat=2");
            if (!m_videoCapture.open(0)) {
                throw std::runtime_error("Error opening video");
            }
        }
        virtual ~CameraSystem() {
        }

        virtual void operateSlow(float dt) { 
            std::lock_guard<std::mutex> lock(m_mutex);
            m_videoCapture >> m_lastFrame;
        }
        
        cv::Mat getLastFrame() {
            std::lock_guard<std::mutex> lock(m_mutex);
            cv::Mat res = m_lastFrame.clone();
            return res;
        }
        cv::Mat &getLastFrameSameThread() { return m_lastFrame; }
    protected:
        std::mutex m_mutex;
        
        cv::VideoCapture m_videoCapture;
        cv::Mat m_lastFrame;
};



class OrbSLAMSystem : public Subsystem
{
    public:
        OrbSLAMSystem(CameraSystem &camera) :
                m_camera(camera),
                m_slam("Vocabulary/ORBvoc.txt", "../RPiCameraWide_640_480.yaml", ORB_SLAM2::System::MONOCULAR, true)
            {
                
            m_startTime = std::chrono::steady_clock::now();
        }
        virtual ~OrbSLAMSystem() {
            m_slam.Shutdown();
        }

        virtual void operateSlow(float dt) { 
            std::lock_guard<std::mutex> lock(m_mutex);
            
            auto t = std::chrono::steady_clock::now();
            double timestamp = std::chrono::duration_cast<std::chrono::duration<double> >(t - m_startTime).count();

            
            auto a = std::chrono::steady_clock::now();
            m_lastEstimatedPose = m_slam.TrackMonocular(m_camera.getLastFrameSameThread(), timestamp);
            auto b = std::chrono::steady_clock::now();
            std::cout << "SLAM.TrackMonocular frame took " << std::chrono::duration_cast<std::chrono::duration<double> >(b - a).count() << " seconds" << std::endl;
        }
        
        cv::Mat getLastPose() {
            std::lock_guard<std::mutex> lock(m_mutex);
            cv::Mat res = m_lastEstimatedPose.clone();
            return res;
        }
        
        cv::Mat &getLastPoseSameThread() {
            return m_lastEstimatedPose;
        }
        
        void renderMap(const char *filename) {
            const std::vector<ORB_SLAM2::MapPoint*> vpMPs = m_slam.getMap()->GetAllMapPoints();
            const std::vector<ORB_SLAM2::MapPoint*> vpRefMPs = m_slam.getMap()->GetReferenceMapPoints();
            
            unsigned width = 1024;
            unsigned height = 1024;
            cv::Mat image = cv::Mat::zeros(width, height, CV_8UC3 );
            
            Eigen::Matrix2f world2imageScale;
            world2imageScale.setIdentity();
            world2imageScale *= width/4.0f;
            Eigen::Vector2f world2imageOffset(width/2, height/2);
            
            for (auto p : vpMPs) {
                if (p->isBad()) continue;
                unsigned numObs = p->Observations();
                cv::Mat pos = p->GetWorldPos();
                cv::Mat normal = p->GetNormal();
                
                Eigen::Vector2f imgPos = world2imageOffset + world2imageScale * Eigen::Vector2f(pos.at<float>(0), pos.at<float>(2));
                Eigen::Vector2f imgNormal = world2imageScale * Eigen::Vector2f(normal.at<float>(0), normal.at<float>(2));
                imgNormal.normalize();
                
                int x = imgPos[0] + 0.5f;
                int y = imgPos[1] + 0.5f;
                
                if ((x < 0) || (x >= width) ||
                    (y < 0) || (y >= height)) continue;
                
                unsigned color_r = 128 + std::min<int>(127, numObs * 20);
                unsigned color_g = 128 + std::min<int>(127, pos.at<float>(1) * 200);
                image.at<cv::Vec3b>(y, x)[0] = color_r;
                image.at<cv::Vec3b>(y, x)[1] = color_g;
                image.at<cv::Vec3b>(y, x)[2] = 0;
                /*
                cv::line(image,
                            cv::Point(x, y),
                            cv::Point(x + imgNormal[0] * 5, y + imgNormal[1] * 5),
                            cv::Scalar( 128, 0, 0 ),
                            1,
                            cv::LINE_4);

                */
            }
            
            {
                cv::Mat pose;
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    pose = m_lastEstimatedPose.clone();
                }

                Eigen::Vector2f camPoints[3];
                camPoints[0] = Eigen::Vector2f(pose.at<float>(0, 3), pose.at<float>(2, 3));
                float angle = std::atan2(-pose.at<float>(2, 0), -pose.at<float>(2, 2));
                float size = 0.1f;
                camPoints[1] = camPoints[0] + Eigen::Vector2f(std::sin(angle - 0.1f) * size, std::cos(angle - 0.1f) * size);
                camPoints[2] = camPoints[0] + Eigen::Vector2f(std::sin(angle + 0.1f) * size, std::cos(angle + 0.1f) * size);
                
                Eigen::Vector2f camImgPoints[3];                
                for (unsigned i = 0; i < 3; i++)
                    camImgPoints[i] = world2imageOffset + world2imageScale * camPoints[i];
                
                cv::line(image,
                            cv::Point(camImgPoints[0][0], camImgPoints[0][1]),
                            cv::Point(camImgPoints[1][0], camImgPoints[1][1]),
                            cv::Scalar( 0, 0, 128 ),
                            1,
                            cv::LINE_4);
                cv::line(image,
                            cv::Point(camImgPoints[0][0], camImgPoints[0][1]),
                            cv::Point(camImgPoints[2][0], camImgPoints[2][1]),
                            cv::Scalar( 0, 0, 128 ),
                            1,
                            cv::LINE_4);
                cv::line(image,
                            cv::Point(camImgPoints[2][0], camImgPoints[2][1]),
                            cv::Point(camImgPoints[1][0], camImgPoints[1][1]),
                            cv::Scalar( 0, 0, 128 ),
                            1,
                            cv::LINE_4);
            }
            
            cv::imwrite(filename, image);
        }
    protected:
        std::chrono::time_point<std::chrono::steady_clock> m_startTime;
        
        CameraSystem &m_camera;
        std::mutex m_mutex;
        
        ORB_SLAM2::System m_slam;
        cv::Mat m_lastEstimatedPose;
};


class Policy : public Subsystem
{
    public:
        Policy(OrbSLAMSystem &slam) :
                m_slam(slam) {
         
            m_drive = false;
        }
        virtual ~Policy() {
        }
        
        void drive() { 
            m_drive = true; 
        }
        void stop() { 
            m_drive = false; 
        }
        
        void addWaypoint() {
            cv::Mat pose = m_slam.getLastPoseSameThread();
            if (pose.empty())
                throw std::runtime_error("No pose estimated!");
            
            Eigen::Vector2f p(pose.at<float>(0, 3), pose.at<float>(2, 3));
            m_waypoints.push_back(p);
            std::cout << "Added waypoint " << p.transpose() << std::endl;
        }

        virtual void operateSlow(float dt) { 
            if (!m_drive) {
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(0.0f, 0.0f);
                return;
            }
            
            cv::Mat &pose = m_slam.getLastPoseSameThread();
            
            if (pose.empty()) {
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(-0.2f, -0.2f);
            } else {
                
                Eigen::Vector2f location(pose.at<float>(0, 3), pose.at<float>(2, 3));
                float angle = std::atan2(-pose.at<float>(2, 0), -pose.at<float>(2, 2));
                
                Eigen::Vector2f targetLocation = m_waypoints[m_currentWaypoint];
                
                Eigen::Vector2f deltaTarget = targetLocation - location;
                float distance = deltaTarget.norm();
                float targetAngle = std::atan2(deltaTarget[0], deltaTarget[1]);
                
                float angleToTarget = targetAngle - angle;
                if (angleToTarget < -M_PI) angleToTarget += 2.0f * M_PI;
                if (angleToTarget > M_PI) angleToTarget -= 2.0f * M_PI;
                
                std::cout << "location: " << location[0] << " " << location[1] << std::endl;
                std::cout << "angle: " << angle << std::endl;
                std::cout << "distance to target: " << distance << std::endl;
                std::cout << "targetAngle: " << angle << std::endl;
                std::cout << "angle to target: " << angleToTarget << std::endl;
                
                if (distance < 0.3f) {
                    std::cout << "######### reached target ###########" << std::endl;
                    
                    if (m_waypoints.size() == 1) {
                        stop();
                    } else {
                        if (m_forward) {
                            m_currentWaypoint++;
                            if (m_currentWaypoint == m_waypoints.size()) {
                                std::cout << "reached end, reversing" << std::endl;
                                m_currentWaypoint = m_waypoints.size()-1;
                                m_forward = false;
                            }
                        } else {
                            m_currentWaypoint--;
                            if (m_currentWaypoint == -1) {
                                std::cout << "reached end, reversing" << std::endl;
                                m_currentWaypoint = 1;
                                m_forward = true;
                            }
                        }
                    }
                } else {
                    Eigen::Vector2f steer;
                    if (angleToTarget < 0.0) 
                        steer = Eigen::Vector2f(0.5f, 0.5f - std::abs(angleToTarget) * 1.0f);
                    else
                        steer = Eigen::Vector2f(0.5f - std::abs(angleToTarget) * 1.0f, 0.5f);
                        
                    std::cout << "Steering :" << steer.transpose() << std::endl;

                    Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(steer[0], steer[1]);
                }
            }
        }
    protected:
        bool m_drive = false;
        bool m_forward = true;
        int m_currentWaypoint = 0;
        std::vector<Eigen::Vector2f> m_waypoints;
        
        OrbSLAMSystem &m_slam;
};

int main()
{
    
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(new DrivePolicyAccLimited()));
    CameraSystem *cameraSystem;
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(cameraSystem = new CameraSystem()));
    OrbSLAMSystem *slam;
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(slam = new OrbSLAMSystem(*cameraSystem)));
    Policy *policy;
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(policy = new Policy(*slam)));
 
    bool shutdown = false;
    while (!shutdown) {
        int c = std::getchar();
        switch (c) {
            case 'a':
                policy->addWaypoint();
            break;
            case 'w':
                policy->drive();
            break;
            case 's':
                policy->stop();
            break;
            case 'q':
                shutdown = true;
            break;
            case ' ':
                slam->renderMap("map.png");
            break;
        }
    }

    
    
    
    return 0;
}
