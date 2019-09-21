#include <robot/WifiCommunication.h>
#include <robot/CameraSystem.h>

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
    
    unsigned steerSequenceNumber = 0;

    Joystick joystick;
    
    if (!joystick.valid())
        std::cout << "Joystick not found!" << std::endl;
    
    robot::SystemMonitoring::State systemState;
    
    while (true) {

        for (unsigned i = 0; i < 10; i++) {
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
                }
            }
        }
        
        cv::flip(frameBuffer, rotatedFramebuffer, -1);
        cv::resize(rotatedFramebuffer, resizedRotatedFramebuffer, cv::Size(1280, 960));
        
        cv::putText(resizedRotatedFramebuffer, 
                    (boost::format("Disk space: %d MB") % systemState.freeSpace_MB).str().c_str(), 
                    cv::Point(0, 10),  cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(1.0f, 1.0f, 1.0f));

        cv::putText(resizedRotatedFramebuffer, 
                    (boost::format("CPU Temperature: %f°C") % (systemState.CPUTemp_mC * 1e-3f)).str().c_str(), 
                    cv::Point(0, 20),  cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(1.0f, 1.0f, 1.0f));
        
        cv::putText(resizedRotatedFramebuffer, 
                    (boost::format("CPU usage: %f%") % (systemState.cpuUsage[0] / 255.0f)).str().c_str(), 
                    cv::Point(0, 30),  cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(1.0f, 1.0f, 1.0f));
        
        for (unsigned i = 0; i < 4; i++) {
            cv::putText(resizedRotatedFramebuffer, 
                        (boost::format("CPU-%d: %f%  %dMHz") % i % (systemState.cpuUsage[i+1] / 255.0f) % (systemState.cpuFreq_KHz[i] / 1000)).str().c_str(), 
                        cv::Point(0, 40 + i*10),  cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(1.0f, 1.0f, 1.0f));
        }
        
        
        cv::imshow("Camera view", resizedRotatedFramebuffer);  
        
        float steerLeft = 0.0f;
        float steerRight = 0.0f;
        
        int key = cv::waitKey(1);
        joystick.poll();
        if (key >= 0) {
            std::cout << "key " << key << std::endl;
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
