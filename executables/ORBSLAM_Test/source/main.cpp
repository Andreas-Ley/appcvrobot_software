#include <robot/Robot.h>
#include <robot/DrivePolicyDirect.h>
#include <robot/WifiCommunication.h>
#include <robot/CameraSystem.h>


#include <System.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <Eigen/Dense>

#include <chrono>
#include <iostream>
#include <cstdio>



std::vector<Eigen::Vector3f> waypoints;
Eigen::Vector3f target(0.0f, 0.0f, 0.0f);


class OrbSLAMSystem : public Subsystem
{
    public:
        OrbSLAMSystem(robot::CameraSystem &camera) :
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
            m_lastPoseValid = m_slam.TrackMonocular(m_camera.getCurrentFrame(), timestamp, m_lastEstimatedPose);
            auto b = std::chrono::steady_clock::now();
            std::cout << "SLAM.TrackMonocular frame took " << std::chrono::duration_cast<std::chrono::duration<double> >(b - a).count() << " seconds" << std::endl;
        }
        
        void getLastPose(Eigen::Matrix4f &pose, bool &valid) {
            std::lock_guard<std::mutex> lock(m_mutex);
            pose = m_lastEstimatedPose;
            valid = m_lastPoseValid;
        }
        
        Eigen::Matrix4f &getLastPoseSameThread() {
            return m_lastEstimatedPose;
        }
        bool getLastPoseValidSameThread() {
            return m_lastPoseValid;
        }
        
        void renderMap(const char *filename) {
            if (m_slam.getMap() != nullptr) {
                const std::vector<ORB_SLAM2::MapPoint*> vpMPs = m_slam.getMap()->GetAllMapPoints();
                const std::vector<ORB_SLAM2::MapPoint*> vpRefMPs = m_slam.getMap()->GetReferenceMapPoints();
                
                unsigned width = 1024;
                unsigned height = 1024;
                cv::Mat image = cv::Mat::zeros(width, height, CV_8UC3 );
                
                Eigen::Matrix<float, 2, 3> world2imageScale;
                world2imageScale.setZero();
                world2imageScale(0, 0) = width/4.0f;
                world2imageScale(1, 2) = width/4.0f;
                Eigen::Vector2f world2imageOffset(width/2, height/2);
                
                for (auto p : vpMPs) {
                    if (p->isBad()) continue;
                    unsigned numObs = p->Observations();
                    Eigen::Vector3f pos = p->GetWorldPos();
                    Eigen::Vector3f normal = p->GetNormal();
                    
                    Eigen::Vector2f imgPos = world2imageOffset + world2imageScale * pos;
                    Eigen::Vector2f imgNormal = world2imageScale * normal;
                    imgNormal.normalize();
                    
                    int x = imgPos[0] + 0.5f;
                    int y = imgPos[1] + 0.5f;
                    
                    if ((x < 0) || (x >= width) ||
                        (y < 0) || (y >= height)) continue;
                    
                    unsigned color_r = 128 + std::min<int>(127, numObs * 20);
                    unsigned color_g = 128 + std::max<int>(-128, std::min<int>(127, pos[2] * 200));
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
                unsigned circleSize = 0.3f * world2imageScale(0, 0);
                Eigen::Vector2f targetImgPos = world2imageOffset + world2imageScale * target;
                cv::circle(image,
                            cv::Point(targetImgPos[0], targetImgPos[1]),
                            circleSize,
                            cv::Scalar( 0, 0, 256 ),
                            1,
                            cv::LINE_4);

                for (auto p : waypoints) {
                    Eigen::Vector2f imgP = world2imageOffset + world2imageScale * p;
                    cv::circle(image,
                                cv::Point(imgP[0], imgP[1]),
                                circleSize,
                                cv::Scalar( 0, 256, 256 ),
                                1,
                                cv::LINE_4);
                }

                
                {
                    Eigen::Matrix4f pose;
                    bool poseValid;
                    {
                        std::lock_guard<std::mutex> lock(m_mutex);
                        pose = m_lastEstimatedPose;
                        poseValid = m_lastPoseValid;
                    }

                    if (poseValid) {
                        Eigen::Vector3f camPoints[3];
                        camPoints[0] = Eigen::Vector3f(-pose(0, 3), -pose(1, 3), -pose(2, 3));
                        float angle = std::atan2(-pose(2, 0), -pose(2, 2));
                        float size = 0.1f;
                        float fov_half = 160 / 2.0f / 180.0f * M_PI;
                        camPoints[1] = camPoints[0] + Eigen::Vector3f(-std::sin(angle - fov_half) * size, 0.0f, -std::cos(angle - fov_half) * size);
                        camPoints[2] = camPoints[0] + Eigen::Vector3f(-std::sin(angle + fov_half) * size, 0.0f, -std::cos(angle + fov_half) * size);
                        
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
                }
                cv::imwrite(filename, image);
            } else {
                unsigned width = 1024;
                unsigned height = 1024;
                cv::Mat image = cv::Mat::zeros(width, height, CV_8UC3 );
                cv::imwrite(filename, image);
            }
        }
    protected:
        std::chrono::time_point<std::chrono::steady_clock> m_startTime;
        
        robot::CameraSystem &m_camera;
        std::mutex m_mutex;
        
        ORB_SLAM2::System m_slam;
        Eigen::Matrix4f m_lastEstimatedPose;
        bool m_lastPoseValid = false;
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
/*
        void addWaypoint() {
            cv::Mat pose = m_slam.getLastPoseSameThread();
            if (pose.empty())
                throw std::runtime_error("No pose estimated!");
            
            Eigen::Vector2f p(-pose.at<float>(0, 3), -pose.at<float>(2, 3));
            m_waypoints.push_back(p);
            std::cout << "Added waypoint " << p.transpose() << std::endl;
        }
*/
        virtual void operateSlow(float dt) { 
            if (!m_drive) {
                Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(0.0f, 0.0f);
                return;
            }
            
            const Eigen::Matrix4f &pose = m_slam.getLastPoseSameThread();
            
            
            if (!m_slam.getLastPoseValidSameThread()) {
                //Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(-0.2f, -0.2f);
                stop();
                std::cout << "Lost tracking!" << std::endl;
            } else {
                Eigen::Vector2f location(-pose(0, 3), -pose(2, 3));
                float angle = std::atan2(pose(2, 0), pose(2, 2));
                
//                Eigen::Vector2f targetLocation = m_waypoints[m_currentWaypoint];
                Eigen::Vector2f targetLocation(target[0], target[2]);
                
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
                    /*
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
                    }*/
                    stop();
                } else {
                    Eigen::Vector2f steer;
                    if (angleToTarget < 0.0) 
                        steer = Eigen::Vector2f(0.4f, std::max(-0.2f, 0.4f - std::abs(angleToTarget) * 0.75f));
                    else
                        steer = Eigen::Vector2f(std::max(-0.2f, 0.4f - std::abs(angleToTarget) * 0.75f), 0.4f);
                        
                    std::cout << "Steering :" << steer.transpose() << std::endl;

                    Robot::robot.getDrivePolicy()->setDesiredWheelSpeed(steer[0], steer[1]);
                }
            }
        }
    protected:
        bool m_drive = false;
        bool m_forward = true;
        int m_currentWaypoint = 0;
        
        OrbSLAMSystem &m_slam;
};

int main()
{
    
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(new DrivePolicyDirect()));

    robot::WifiCommunication *wifiCom;
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(wifiCom = new robot::WifiCommunication(1337)));

    robot::CameraSystem *cameraSystem;
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(cameraSystem = new robot::CameraSystem(wifiCom, 500'000))); // 500kB/s
    OrbSLAMSystem *slam;
    Policy *policy;
    /*
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(slam = new OrbSLAMSystem(*cameraSystem)));
    Robot::robot.addSubsystem(std::unique_ptr<Subsystem>(policy = new Policy(*slam)));
    */
 
    bool shutdown = false;
    while (!shutdown) {
        int c = std::getchar();
        switch (c) {
            case 'a':
                try {
                    Eigen::Matrix4f pose;
                    bool poseValid;
                    slam->getLastPose(pose, poseValid);
                    if (!poseValid)
                        throw std::runtime_error("No pose estimated!");
                    
                    Eigen::Vector3f p(-pose(0, 3), -pose(1, 3), -pose(2, 3));
                    waypoints.push_back(p);

                } catch (const std::exception &e) {
                    std::cout << e.what() << std::endl;
                }
            break;
            case 't':
                std::cin >> target[0];
                std::cin >> target[2];
                std::cout << "Target set to " << target.transpose() << std::endl;
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
                cv::Mat camImg = cameraSystem->getCurrentFrame();
                cv::imwrite("camera.png", camImg);
            break;
        }
    }

    
    
    
    return 0;
}
