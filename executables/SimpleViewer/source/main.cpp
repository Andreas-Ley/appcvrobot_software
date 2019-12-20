#include <robot/WifiCommunication.h>
#include <robot/CameraSystem.h>
#ifdef BUILD_WITH_ORB_SLAM
#include <robot/SLAMSystem.h>
#endif

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/format.hpp>

#include <iostream>
#include <cstdio>
#include <limits>



#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

struct js_event {
    std::uint32_t time;     /* event timestamp in milliseconds */
    std::int16_t value;    /* value */
    std::uint8_t type;      /* event type */
    std::uint8_t number;    /* axis/button number */
};

class Joystick
{
    public:
        Joystick(const char *deviceFile = "/dev/input/js0") {
            m_fd = open(deviceFile, O_RDONLY | O_NONBLOCK);
            
            axis[0] = 0.0f;
            axis[1] = 0.0f;
        }
        ~Joystick() {
            if (m_fd > 0)
                close(m_fd);
        }
        
        bool valid() { return m_fd > 0; }
        
        void poll() {
            if (m_fd <= 0)
                return;
            js_event e;
            while (read(m_fd, &e, sizeof(e)) > 0) {
                switch (e.type) {
                    case JS_EVENT_AXIS: {
                        std::cout << e.number << std::endl;
                        switch (e.number) {
                            case 0:
                                axis[0] = e.value / (float) std::numeric_limits<std::int16_t>::max();
                            break;
                            case 1:
                                axis[1] = e.value / (float) std::numeric_limits<std::int16_t>::max();
                            break;
                        }
                    }
                }
            }
        }
        
        float axis[2];
    protected:
        int m_fd;        
};



int main(int argc, char **argv)
{
    using boost::asio::ip::udp;
    
    boost::asio::io_service io_service;

    boost::system::error_code error;
    auto targetIP = boost::asio::ip::address::from_string(argv[1], error); 
    
    
    udp::endpoint receiver_endpoint(targetIP, 1337);
    
    udp::socket socket(io_service);
    socket.open(udp::v4());
    
    std::string message = "I, for one, welcome our new machine overlords";
    socket.send_to(boost::asio::buffer(message), receiver_endpoint);
    
    cv::Mat frameBuffer(640, 480, CV_8UC3, cv::Scalar::all(0));
    cv::Mat rotatedFramebuffer, resizedRotatedFramebuffer;
     
    cv::namedWindow( "Camera view", cv::WINDOW_AUTOSIZE );
    
#ifdef BUILD_WITH_ORB_SLAM
    cv::namedWindow( "SLAM map", cv::WINDOW_AUTOSIZE );
    
    cv::Mat slamMapFrameBuffer(960, 1280, CV_8UC3, cv::Scalar::all(0));
    
#endif
    
    unsigned steerSequenceNumber = 0;

    Joystick joystick;
    
    if (!joystick.valid())
        std::cout << "Joystick not found!" << std::endl;
    
    robot::SystemMonitoring::State systemState;
    
#ifdef BUILD_WITH_ORB_SLAM
    Eigen::Matrix4f lastSLAMPose = Eigen::Matrix4f::Identity();
    bool slamTrackingLost = true;
    unsigned lastSLAMPoseSeqNumber = 0;
    
    float slamMapRotation = 0.0f;
    
    Eigen::Matrix<float, 3, 4> slamMapProjMatrix;
    slamMapProjMatrix.setZero();
    slamMapProjMatrix(0, 0) = 400.0f;
    slamMapProjMatrix(1, 1) = -400.0f;
    slamMapProjMatrix(2, 3) = 1.0f;
    slamMapProjMatrix(0, 3) = slamMapFrameBuffer.cols*0.5f;
    slamMapProjMatrix(1, 3) = slamMapFrameBuffer.rows*0.5f;
    
    std::vector<robot::SLAMSystem::NetworkMapSlice::Point> slamMap;
#endif
    
    while (true) {

        for (unsigned i = 0; i < 30; i++) {
            boost::array<char, 2048> recv_buf;
            udp::endpoint sender_endpoint;

            size_t len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
            // check correct sender...
            
            if (len < sizeof(robot::WifiCommunication::CameraTilePacket))
                continue;
            
            
            const robot::WifiCommunication::MessageHeader *header = (const robot::WifiCommunication::MessageHeader *) recv_buf.data();

            switch (header->messageType) {
                case robot::WifiCommunication::MESSAGE_ID_CAMERA_TILE: {
                    const robot::WifiCommunication::CameraTilePacket *packet = (const robot::WifiCommunication::CameraTilePacket *) recv_buf.data();
                    unsigned dataLen = len - sizeof(robot::WifiCommunication::CameraTilePacket);
                    
                    std::vector<unsigned char> imgBuffer(packet->data, packet->data+dataLen);
                    cv::Mat tile = cv::imdecode(imgBuffer, cv::IMREAD_COLOR);
                    
                    if (tile.empty()) {
                        std::cout << "Error: Could not decode image tile!" << std::endl;
                        continue;
                    }
                    
                    if ((packet->tileSpec.frameW != frameBuffer.cols) || (packet->tileSpec.frameH != frameBuffer.rows)) {
                        frameBuffer.create(packet->tileSpec.frameH, packet->tileSpec.frameW, CV_8UC3);
                    }
                    // todo: check validity
                    tile.copyTo(frameBuffer
                                    .rowRange(packet->tileSpec.y, packet->tileSpec.y + packet->tileSpec.h)
                                    .colRange(packet->tileSpec.x, packet->tileSpec.x + packet->tileSpec.w));
                } break;
                case robot::WifiCommunication::MESSAGE_ID_SYSTEM_MONITORING: {
                    const robot::WifiCommunication::SystemMonitoringPacket *packet = (const robot::WifiCommunication::SystemMonitoringPacket *) recv_buf.data();
                    systemState = packet->state;
                } break;
#ifdef BUILD_WITH_ORB_SLAM
                case robot::WifiCommunication::MESSAGE_ID_SLAM_POSE: {
                    const robot::WifiCommunication::SLAMPosePacket *packet = (const robot::WifiCommunication::SLAMPosePacket *) recv_buf.data();
                    if (packet->header.sequenceNumber < lastSLAMPoseSeqNumber) break;
                    
                    lastSLAMPoseSeqNumber = packet->header.sequenceNumber;
                    
                    Eigen::Quaternionf rot;
                    rot.x() = packet->pose.qx;
                    rot.y() = packet->pose.qy;
                    rot.z() = packet->pose.qz;
                    rot.w() = packet->pose.qw;
                    
                    // todo: check sequence number!!!

                    lastSLAMPose.block<3, 3>(0, 0) = rot.toRotationMatrix();
                    lastSLAMPose(0, 3) = packet->pose.px;
                    lastSLAMPose(1, 3) = packet->pose.py;
                    lastSLAMPose(2, 3) = packet->pose.pz;
                    
                    slamTrackingLost = packet->pose.trackingLost;
                } break;
                case robot::WifiCommunication::MESSAGE_ID_SLAM_MAP_SLICE: {
                    const robot::WifiCommunication::SLAMMapSlicePacket *packet = (const robot::WifiCommunication::SLAMMapSlicePacket *) recv_buf.data();

                    slamMap.resize(packet->mapSlice.numPointsTotal);
                    if (slamMap.size() > 0) {
                        unsigned dstIdx = packet->mapSlice.sliceOffset % slamMap.size();
                        for (unsigned i = 0; i < packet->mapSlice.sliceCount; i++) {
                            slamMap[dstIdx] = packet->mapSlice.points[i];
                            dstIdx++;
                            if (dstIdx == slamMap.size())
                                dstIdx = 0;
                        }
                    }
                } break;
#endif
            }
        }
        
        cv::flip(frameBuffer, rotatedFramebuffer, -1);
        cv::resize(rotatedFramebuffer, resizedRotatedFramebuffer, cv::Size(1280, 960));
        
        cv::putText(resizedRotatedFramebuffer, 
                    (boost::format("Disk space: %d MB") % systemState.freeSpace_MB).str().c_str(), 
                    cv::Point(0, 30),  cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255, 255, 255));

        cv::putText(resizedRotatedFramebuffer, 
                    (boost::format("RAM avail: %d MB of %d MB") % (systemState.memAvailable_KB/1024) % (systemState.memTotal_KB/1024)).str().c_str(), 
                    cv::Point(0, 60),  cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255, 255, 255));

        cv::putText(resizedRotatedFramebuffer, 
                    (boost::format("CPU Temperature: %f C") % (systemState.CPUTemp_mC * 1e-3f)).str().c_str(), 
                    cv::Point(0, 90),  cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255, 255, 255));
        
        cv::putText(resizedRotatedFramebuffer, 
                    (boost::format("CPU usage: %f%%") % (systemState.cpuUsage[0] * 100.0f / 255.0f)).str().c_str(), 
                    cv::Point(0, 120),  cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255, 255, 255));
        
        for (unsigned i = 0; i < 4; i++) {
            cv::putText(resizedRotatedFramebuffer, 
                        (boost::format("CPU-%d: %f%%  %dMHz") % i % (systemState.cpuUsage[i+1] * 100.0f / 255.0f) % (systemState.cpuFreq_KHz[i] / 1000)).str().c_str(), 
                        cv::Point(0, 150 + i*30),  cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255, 255, 255));
        }

        cv::putText(resizedRotatedFramebuffer, 
                    (boost::format("Cell 1: %d V") % (systemState.cellVoltage_div20[0] / 20.0f)).str().c_str(), 
                    cv::Point(900, 30),  cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255, 255, 255));
        
        cv::putText(resizedRotatedFramebuffer, 
                    (boost::format("Battery draw: %d A") % (systemState.batteryDrawAmps_div100 / 100.0f)).str().c_str(), 
                    cv::Point(900, 60),  cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255, 255, 255));
        
        cv::imshow("Camera view", resizedRotatedFramebuffer);  
        
        
        
        
#ifdef BUILD_WITH_ORB_SLAM
        slamMapFrameBuffer.setTo(cv::Scalar(0, 0, 0));
        
        slamMapRotation += 0.01f;
        if (slamMapRotation > M_PI*2.0f)
            slamMapRotation -= M_PI*2.0f;
        
        Eigen::Matrix4f slamMapViewMatrix;
        slamMapViewMatrix.setIdentity();
        slamMapViewMatrix.block<3, 3>(0, 0) = (
                Eigen::AngleAxis<float>(70.0f / 180.0f * M_PI, Eigen::Vector3f(1.0f, 0.0f, 0.0f)) * 
                Eigen::AngleAxis<float>(slamMapRotation, Eigen::Vector3f(0.0f, 1.0f, 0.0f))
            ).matrix();
        
        Eigen::Matrix<float, 3, 4> MVP = slamMapProjMatrix * slamMapViewMatrix;
        
        for (auto &p : slamMap) {
            Eigen::Vector3f imgH = MVP * Eigen::Vector4f(p.x, p.y, p.z, 1.0f);
            Eigen::Vector2f img = imgH.head<2>() / imgH[2];
            
            int x = img[0];
            int y = img[1];
            if ((x > 0) && (x < slamMapFrameBuffer.cols) &&
                (y > 0) && (y < slamMapFrameBuffer.rows))
                slamMapFrameBuffer.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
        }
        
        auto drawLineWS = [&](const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const cv::Scalar &color) {
            Eigen::Vector3f img1H = MVP * Eigen::Vector4f(p1[0], p1[1], p1[2], 1.0f);
            Eigen::Vector2f img1 = img1H.head<2>() / img1H[2];

            Eigen::Vector3f img2H = MVP * Eigen::Vector4f(p2[0], p2[1], p2[2], 1.0f);
            Eigen::Vector2f img2 = img2H.head<2>() / img2H[2];
            
            cv::line(slamMapFrameBuffer,
                        cv::Point(img1[0], img1[1]),
                        cv::Point(img2[0], img2[1]),
                        color,
                        1,
                        cv::LINE_4);
        };
        
        Eigen::Vector3f cameraCenter = - lastSLAMPose.block<3, 3>(0, 0).transpose() * lastSLAMPose.block<3, 1>(0, 3);
        
        auto R = lastSLAMPose.block<3, 3>(0, 0).transpose();
        float w = std::tan(160.0f/2.0f/180.0f*M_PI);
        
        Eigen::Vector3f frustrum[4] = {
            cameraCenter + R * Eigen::Vector3f(-w, -w, 1.0f) * 0.01f,
            cameraCenter + R * Eigen::Vector3f(w, -w, 1.0f) * 0.01f,
            cameraCenter + R * Eigen::Vector3f(w, w, 1.0f) * 0.01f,
            cameraCenter + R * Eigen::Vector3f(-w, w, 1.0f) * 0.01f
        };
        
        drawLineWS(cameraCenter, frustrum[0], cv::Scalar(255, 0, 0));
        drawLineWS(cameraCenter, frustrum[1], cv::Scalar(255, 0, 0));
        drawLineWS(cameraCenter, frustrum[2], cv::Scalar(255, 0, 0));
        drawLineWS(cameraCenter, frustrum[3], cv::Scalar(255, 0, 0));
        drawLineWS(frustrum[0], frustrum[1], cv::Scalar(255, 0, 0));
        drawLineWS(frustrum[1], frustrum[2], cv::Scalar(255, 0, 0));
        drawLineWS(frustrum[2], frustrum[3], cv::Scalar(255, 0, 0));
        drawLineWS(frustrum[3], frustrum[0], cv::Scalar(255, 0, 0));
        
        drawLineWS(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(1.0f, 0.0f, 0.0f), cv::Scalar(0, 0, 255));
        drawLineWS(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f), cv::Scalar(0, 255, 0));
        drawLineWS(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f), cv::Scalar(255, 0, 0));
        
        if (slamTrackingLost)
            cv::putText(slamMapFrameBuffer, 
                        "Tracking lost!", 
                        cv::Point(0, 60),  cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(255, 255, 255));

        cv::imshow("SLAM map", slamMapFrameBuffer);  
#endif

        
        
        
        
        
        float steerLeft = 0.0f;
        float steerRight = 0.0f;
        
        int key = cv::waitKey(1);
        joystick.poll();
        if (key >= 0) {
            //std::cout << "key " << key << std::endl;
            switch (key) {
                case 27:
                    return 0;
                break;
                case 'w': 
                    steerLeft = 1.0f;
                    steerRight = 1.0f;
                break;
                case 's':
                    steerLeft = -1.0f;
                    steerRight = -1.0f;
                break;
                case 'a':
                    steerLeft = -1.0f;
                    steerRight = 1.0f;
                break;
                case 'd':
                    steerLeft = 1.0f;
                    steerRight = -1.0f;
                break;
                case 'q':
                    steerLeft = 0.5f;
                    steerRight = 1.0f;
                break;
                case 'e':
                    steerLeft = 1.0f;
                    steerRight = 0.5f;
                break;
            };
        }
        
        if (!joystick.valid()) {
            steerLeft *= 1.0f;
            steerRight *= 1.0f;
        } else {
            steerLeft = -joystick.axis[1] + joystick.axis[0] * 0.4f;
            steerRight = -joystick.axis[1] - joystick.axis[0] * 0.4f;
        }
        
        steerLeft = std::max(std::min(steerLeft, 1.0f), -1.0f);
        steerRight = std::max(std::min(steerRight, 1.0f), -1.0f);
        
        robot::WifiCommunication::RemoteSteerCmdPacket steerPacket;
        steerPacket.header.messageType = robot::WifiCommunication::MESSAGE_ID_REMOTE_STEER_CMD;
        steerPacket.header.sequenceNumber = steerSequenceNumber++;
        steerPacket.left = steerLeft * std::numeric_limits<decltype(steerPacket.left)>::max();
        steerPacket.right = steerRight * std::numeric_limits<decltype(steerPacket.left)>::max();
        
        socket.send_to(boost::asio::buffer(&steerPacket, sizeof(steerPacket)), receiver_endpoint);
    }
   
    return 0;
}
