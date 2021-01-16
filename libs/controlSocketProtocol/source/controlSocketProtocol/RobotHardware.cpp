#include "RobotHardware.h"

namespace robot
{
    RobotHardware::RobotHardware(boost::asio::io_context &ioContext) : m_socket(ioContext)
    {
        const char *socketFile = "/var/run/robothardwarecontroller.socket";
        m_socket.connect(boostSockProt::endpoint(socketFile));
    }

    hardwareSocket::ResponseBodyCellVoltages RobotHardware::getBatteryCellVoltages()
    {
        robot::hardwareSocket::RequestCodes requestCode = robot::hardwareSocket::RequestCodes::BATTERY_CELL_VOLTAGES;
        struct
        {
            robot::hardwareSocket::ResponseCodes responseCode;
            robot::hardwareSocket::ResponseBodyCellVoltages body;
        } voltageResponse;
        memset(&voltageResponse, 0, sizeof(voltageResponse));

        boost::asio::write(m_socket, boost::asio::buffer(&requestCode, sizeof(requestCode)));
        boost::asio::read(m_socket, boost::asio::buffer(&voltageResponse, sizeof(voltageResponse)));

        if (voltageResponse.responseCode != robot::hardwareSocket::ResponseCodes::OK)
        {
            throw std::runtime_error("Getting voltages failed.");
        }
        return voltageResponse.body;
    }

    float RobotHardware::getCurrentDraw()
    {
        robot::hardwareSocket::RequestCodes requestCode = robot::hardwareSocket::RequestCodes::CURRENT_DRAW;
        struct
        {
            robot::hardwareSocket::ResponseCodes responseCode;
            robot::hardwareSocket::ResponseBodyCurrent body;
        } response;
        memset(&response, 0, sizeof(response));

        boost::asio::write(m_socket, boost::asio::buffer(&requestCode, sizeof(requestCode)));
        boost::asio::read(m_socket, boost::asio::buffer(&response, sizeof(response)));

        if (response.responseCode != robot::hardwareSocket::ResponseCodes::OK)
        {
            throw std::runtime_error("Getting current failed.");
        }
        return response.body.current;
    }

    float RobotHardware::getControllerUsage()
    {
        robot::hardwareSocket::RequestCodes requestCode = robot::hardwareSocket::RequestCodes::CONTOLLER_USAGE;
        struct
        {
            robot::hardwareSocket::ResponseCodes responseCode;
            robot::hardwareSocket::ResponseBodyCpu body;
        } response;
        memset(&response, 0, sizeof(response));

        boost::asio::write(m_socket, boost::asio::buffer(&requestCode, sizeof(requestCode)));
        boost::asio::read(m_socket, boost::asio::buffer(&response, sizeof(response)));

        if (response.responseCode != robot::hardwareSocket::ResponseCodes::OK)
        {
            throw std::runtime_error("Getting controller usage failed.");
        }
        return response.body.cpu;
    }

    void RobotHardware::acquireDrive()
    {
        robot::hardwareSocket::RequestCodes requestCode = robot::hardwareSocket::RequestCodes::DRIVE_ACQUIRE;
        robot::hardwareSocket::ResponseCodes responseCode;

        boost::asio::write(m_socket, boost::asio::buffer(&requestCode, sizeof(requestCode)));
        boost::asio::read(m_socket, boost::asio::buffer(&responseCode, sizeof(responseCode)));

        switch (responseCode)
        {
        case robot::hardwareSocket::ResponseCodes::DRIVE_ALREADY_ACQUIRED:
            throw std::runtime_error("Drive was already acquired by different process.");

        case robot::hardwareSocket::ResponseCodes::OK:
            return;

        default:
            throw std::runtime_error("Acquiring drive failed.");
        }
    }

    void RobotHardware::releaseDrive()
    {
        robot::hardwareSocket::RequestCodes requestCode = robot::hardwareSocket::RequestCodes::DRIVE_RELEASE;
        robot::hardwareSocket::ResponseCodes responseCode;

        boost::asio::write(m_socket, boost::asio::buffer(&requestCode, sizeof(requestCode)));
        boost::asio::read(m_socket, boost::asio::buffer(&responseCode, sizeof(responseCode)));

        switch (responseCode)
        {
        case robot::hardwareSocket::ResponseCodes::DRIVE_WAS_NOT_ACQUIRED:
            throw std::runtime_error("Drive was not acquired by this process.");

        case robot::hardwareSocket::ResponseCodes::OK:
            return;

        default:
            throw std::runtime_error("Releasing drive failed.");
        }
    }

    void RobotHardware::setDriveSpeed(float left, float right)
    {
        struct
        {
            robot::hardwareSocket::RequestCodes requestCode;
            robot::hardwareSocket::RequestBodyDriveSetSpeed body;
        } request;
        memset(&request, 0, sizeof(request));

        request.requestCode = robot::hardwareSocket::RequestCodes::DRIVE_SET_SPEED;

        request.body.speedLeft = left;
        request.body.speedRight = right;

        boost::asio::write(m_socket, boost::asio::buffer(&request, sizeof(request)));

        robot::hardwareSocket::ResponseCodes response;
        boost::asio::read(m_socket, boost::asio::buffer(&response, sizeof(response)));

        switch (response)
        {
        case robot::hardwareSocket::ResponseCodes::DRIVE_WAS_NOT_ACQUIRED:
            throw std::runtime_error("Setting drive speed failed: drive was not acquired by this process.");

        case robot::hardwareSocket::ResponseCodes::OK:
            return;

        default:
            throw std::runtime_error("Setting drive speed failed.");
        }
    }

    std::tuple<int, int> RobotHardware::getDriveSteps()
    {
        robot::hardwareSocket::RequestCodes requestCode = robot::hardwareSocket::RequestCodes::DRIVE_GET_STEPS;
        struct
        {
            robot::hardwareSocket::ResponseCodes responseCode;
            robot::hardwareSocket::ResponseBodyDriveGetSteps body;
        } response;
        memset(&response, 0, sizeof(response));

        boost::asio::write(m_socket, boost::asio::buffer(&requestCode, sizeof(requestCode)));
        boost::asio::read(m_socket, boost::asio::buffer(&response, sizeof(response)));

        if (response.responseCode != robot::hardwareSocket::ResponseCodes::OK)
        {
            throw std::runtime_error("Getting drive steps failed.");
        }
        return {(int)response.body.stepsLeft, (int)response.body.stepsRight};
    }

    void RobotHardware::setLcdText(const std::string &s1, const std::string &s2)
    {
        struct
        {
            robot::hardwareSocket::RequestCodes requestCode;
            robot::hardwareSocket::RequestBodyLCDSetText body;
        } request;
        memset(&request, 0, sizeof(request));

        request.requestCode = robot::hardwareSocket::RequestCodes::LCD_SET_TEXT;

        request.body.setLine(0, s1);
        request.body.setLine(1, s2);

        boost::asio::write(m_socket, boost::asio::buffer(&request, sizeof(request)));

        robot::hardwareSocket::ResponseCodes response;
        boost::asio::read(m_socket, boost::asio::buffer(&response, sizeof(response)));

        if (response != robot::hardwareSocket::ResponseCodes::OK)
        {
            throw std::runtime_error("Setting LCD text failed.");
        }
    }

    void RobotHardware::acquireLcd() {}
    void RobotHardware::releaseLcd() {}

    hardwareSocket::ResponseBodyButtons RobotHardware::getButtonsPushed()
    {
        robot::hardwareSocket::RequestCodes requestCode = robot::hardwareSocket::RequestCodes::BUTTONS_PUSHED;
        struct
        {
            robot::hardwareSocket::ResponseCodes responseCode;
            robot::hardwareSocket::ResponseBodyButtons body;
        } response;
        memset(&response, 0, sizeof(response));

        boost::asio::write(m_socket, boost::asio::buffer(&requestCode, sizeof(requestCode)));
        boost::asio::read(m_socket, boost::asio::buffer(&response, sizeof(response)));

        if (response.responseCode != robot::hardwareSocket::ResponseCodes::OK)
        {
            throw std::runtime_error("Getting pushed buttons failed.");
        }
        return {response.body};
    }
} // namespace robot