#include <robot/WifiCommunication.h>

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
    while (true) {

        for (unsigned i = 0; i < 10; i++) {
            boost::array<char, 2048> recv_buf;
            udp::endpoint sender_endpoint;

            size_t len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
            // check correct sender...
            
            struct TileSpec {
                std::uint32_t frameW, frameH;
                std::uint32_t x, y, w, h;
            };

            if (len < 4 + sizeof(TileSpec))
                continue;
            
            std::uint32_t messageId;
            TileSpec tileSpec;
            std::vector<unsigned char> imgBuffer;
            imgBuffer.resize(len - 4 + sizeof(TileSpec));
                
            unsigned offset = 0;
            memcpy(&messageId, &recv_buf[offset], sizeof(messageId)); offset += sizeof(messageId);
            memcpy(&tileSpec, &recv_buf[offset], sizeof(tileSpec)); offset += sizeof(tileSpec);
            memcpy(imgBuffer.data(), &recv_buf[offset], imgBuffer.size()); offset += imgBuffer.size();

            cv::Mat tile = cv::imdecode(imgBuffer, cv::IMREAD_COLOR);
            
            if (tile.empty()) {
                std::cout << "Error: Could not decode image tile!" << std::endl;
                continue;
            }
            
            std::cout << tileSpec.frameW << " " << tileSpec.frameH << std::endl;
            std::cout << tileSpec.x << " " << tileSpec.y << " " << tileSpec.w << " " << tileSpec.h << std::endl;
            
            if ((tileSpec.frameW != frameBuffer.cols) || (tileSpec.frameH != frameBuffer.rows)) {
                frameBuffer.create(tileSpec.frameH, tileSpec.frameW, CV_8UC3);
            }
            // todo: check validity
            tile.copyTo(frameBuffer.rowRange(tileSpec.y, tileSpec.y + tileSpec.h).colRange(tileSpec.x, tileSpec.x + tileSpec.w));
        }
        
        cv::imshow("Camera view", frameBuffer);  
        cv::waitKey(1);
    }
   
    return 0;
}
