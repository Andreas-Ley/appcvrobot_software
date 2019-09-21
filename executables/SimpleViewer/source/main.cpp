#include <robot/WifiCommunication.h>
#include <robot/CameraSystem.h>

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <boost/array.hpp>
#include <boost/asio.hpp>

#include <iostream>
#include <cstdio>

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
     
    cv::namedWindow( "Camera view", cv::WINDOW_AUTOSIZE );
    
    unsigned steerSequenceNumber = 0;
    
    while (true) {

        for (unsigned i = 0; i < 2; i++) {
            boost::array<char, 2048> recv_buf;
            udp::endpoint sender_endpoint;

            size_t len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
            // check correct sender...
            
            if (len < sizeof(robot::WifiCommunication::CameraTilePacket))
                continue;
            
            
            const robot::WifiCommunication::CameraTilePacket *packet = (const robot::WifiCommunication::CameraTilePacket *) recv_buf.data();

        
            if (packet->header.messageType != robot::WifiCommunication::MESSAGE_ID_CAMERA_TILE)
                continue;
            
            unsigned dataLen = len - sizeof(robot::WifiCommunication::CameraTilePacket);
            
            std::vector<unsigned char> imgBuffer(packet->data, packet->data+dataLen);
            std::cout << imgBuffer.size() << std::endl;
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
        }
        
        cv::imshow("Camera view", frameBuffer);  
        
        float steerLeft = 0.0f;
        float steerRight = 0.0f;
        
        int key = cv::waitKey(1);
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
        steerLeft *= 1.0f;
        steerRight *= 1.0f;
        
        robot::WifiCommunication::RemoteSteerCmdPacket steerPacket;
        steerPacket.header.messageType = robot::WifiCommunication::MESSAGE_ID_REMOTE_STEER_CMD;
        steerPacket.header.sequenceNumber = steerSequenceNumber++;
        steerPacket.left = steerLeft * std::numeric_limits<decltype(steerPacket.left)>::max();
        steerPacket.right = steerRight * std::numeric_limits<decltype(steerPacket.left)>::max();
        
        socket.send_to(boost::asio::buffer(&steerPacket, sizeof(steerPacket)), receiver_endpoint);
    }
   
    return 0;
}
