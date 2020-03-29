/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2020  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Session.h"

#include "ControlSocket.h"






#include <boost/bind.hpp>

#include <iostream>



Session::Session(ControlSocket &controlSocket) : m_controlSocket(controlSocket), m_socket(controlSocket.getIOContext()) 
{
}

void Session::startRecvRequestHead()
{
    boost::asio::async_read(m_socket,
        boost::asio::buffer(&m_request.head, sizeof(m_request.head)),
        boost::bind(&Session::onRequestHeadRecvd, this, boost::asio::placeholders::error));
}

void Session::onRequestHeadRecvd(const boost::system::error_code& error)
{
    if (error) {
        // todo: log
        m_controlSocket.dropSession(this);
    } else {
        switch (m_request.head) {
								//BATERY
            case robot::hardwareSocket::RequestCodes::BATTERY_CELL_VOLTAGES:
                onRequestBodyRecvd(boost::system::error_code()); // skip body recieving and directly go to evaluation
            break;
			case robot::hardwareSocket::RequestCodes::CURRENT_DRAW:
                onRequestBodyRecvd(boost::system::error_code()); // skip body recieving and directly go to evaluation
            break;
			case robot::hardwareSocket::RequestCodes::CONTOLLER_USAGE:
                onRequestBodyRecvd(boost::system::error_code()); // skip body recieving and directly go to evaluation
            break;
								//DRIVE
			case robot::hardwareSocket::RequestCodes::DRIVE_ACQUIRE :
				 if(robot::hardwareSocket::MotorAq==true){	//global
					robot::hardwareSocket::MotorAq=false;
					MotorAq=true;							//local
				 }
				 else(
				 Sleep(100);										// wait 100 ms and try again
				 onRequestHeadRecvd(boost::system::error_code())
				 )
                
            break;
			case robot::hardwareSocket::RequestCodes::DRIVE_RELEASE:
				if(MotorAq==true){
					MotorAq=false;
					robot::hardwareSocket::MotorAq=true;
				}
				
            break;
            case robot::hardwareSocket::RequestCodes::DRIVE_SET_SPEED:
				if(MotorAq==true){
                startRecvRequestBody(sizeof(m_request.body.driveSetSpeed));
				}
				
				
            break;
			case robot::hardwareSocket::RequestCodes::DRIVE_GET_STEPS:
                onRequestBodyRecvd(boost::system::error_code()); // skip body recieving and directly go to evaluation
            break;
									//LCD
			case robot::hardwareSocket::RequestCodes::LCD_ACQUIRE:
				if(robot::hardwareSocket::LcdAq==true){
					robot::hardwareSocket::LcdAq=false;
					LcdAq=true;
				 }
				 else(
				 Sleep(100);										// wait 100ms and try again
				 onRequestHeadRecvd(boost::system::error_code())
				 )  
            break;
			case robot::hardwareSocket::RequestCodes::LCD_RELEASE :
				if(LcdAq==true){
					LcdAq=false;
					robot::hardwareSocket::LcdAq=true;
				}
				else{
					//log
            break;
			case robot::hardwareSocket::RequestCodes::LCD_SET_TEXT:
				if(LcdAq==true){
                startRecvRequestBody(sizeof(m_request.body.LCDSetText));
				}
				else{
					
				}
            break;
								//BUTTONS
			case robot::hardwareSocket::RequestCodes::BUTTONS_PUSHED :
                onRequestBodyRecvd(boost::system::error_code()); // skip body recieving and directly go to evaluation
            break;
            
			
            default:
                m_response.head = robot::hardwareSocket::ResponseCodes::UNKNOWN_REQUEST;
                startSendFailureCode();
            break;
        }
    }
}

void Session::startRecvRequestBody(std::size_t bodySize)
{
    boost::asio::async_read(m_socket,
        boost::asio::buffer(&m_request.body, bodySize),
        boost::bind(&Session::onRequestBodyRecvd, this, boost::asio::placeholders::error));
}

void Session::onRequestBodyRecvd(const boost::system::error_code& error)
{
    if (error) {
        // todo: log
        m_controlSocket.dropSession(this);
    } else {
        switch (m_request.head) {
									//BATERY
            case robot::hardwareSocket::RequestCodes::BATTERY_CELL_VOLTAGES:
                m_response.head = robot::hardwareSocket::ResponseCodes::OK;
                m_response.body.cellVoltages.voltages[0] = hardwareInterface::battery::getCellVoltage(0);
                m_response.body.cellVoltages.voltages[1] = hardwareInterface::battery::getCellVoltage(1);
                m_response.body.cellVoltages.voltages[2] = hardwareInterface::battery::getCellVoltage(2);
                startSendResponse(sizeof(m_response.body.cellVoltages));
            break;
            
			case robot::hardwareSocket::RequestCodes::CURRENT_DRAW:
                m_response.head = robot::hardwareSocket::ResponseCodes::OK;
                m_response.body.current.Current = hardwareInterface::battery::getBatteryCurrentAmps();
                startSendResponse(sizeof(m_response.body.current));
            break;
			
			
			case robot::hardwareSocket::RequestCodes::CONTOLLER_USAGE:
                m_response.head = robot::hardwareSocket::ResponseCodes::OK;
                m_response.body.cpu.Cpu = hardwareInterface::motors::getControllerCPUUsage();
  
                startSendResponse(sizeof(m_response.body.cpu));
            break;
			
							//DRIVE
			
			case robot::hardwareSocket::RequestCodes::DRIVE_GET_STEPS:
                m_response.head = robot::hardwareSocket::ResponseCodes::OK;
				std::int16_t left;
				std::int16_t right;
				hardwareInterface::battery::getCellVoltage(&left,&right);
                m_response.body.driveGetSteps.stepsLeft = left;
                m_response.body.driveGetSteps.stepsRight = right;
                startSendResponse(sizeof(m_response.body.driveGetSteps));
            break;
			
			
			case robot::hardwareSocket::RequestCodes::DRIVE_SET_SPEED:
                m_response.head = robot::hardwareSocket::ResponseCodes::OK;
                hardwareInterface::motors::setSpeed( m_response.body.driveSetSpeed.speedLeft, m_response.body.driveSetSpeed.speedRight));
                startSendResponse(0);
            break;
										//LCD
			case robot::hardwareSocket::RequestCodes::LCD_SET_TEXT:
               


                // sanitize:
                for (unsigned i = 0; i < robot::hardwareSocket::RequestBodyLCDSetText::NUM_LINES; i++) {
                    for (unsigned j = 0; j < robot::hardwareSocket::RequestBodyLCDSetText::LINE_LENGTH; j++)
                        m_request.body.LCDSetText.lines[i][j] &= 0b01111111;
                    m_request.body.LCDSetText.lines[i][robot::hardwareSocket::RequestBodyLCDSetText::LINE_LENGTH] = 0x00;
                }
                
                
                //std::cout << "LCD set text:" << std::endl;
                //std::cout << m_request.body.LCDSetText.lines[0] << std::endl;
                //std::cout << m_request.body.LCDSetText.lines[1] << std::endl;
				hardwareInterface::lcd::writeLine(m_request.body.LCDSetText.lines[0],0);
				hardwareInterface::lcd::writeLine(m_request.body.LCDSetText.lines[1],1);
                m_response.head = robot::hardwareSocket::ResponseCodes::OK;
                startSendResponse(0);
            break;
								//BUTTONS
			
			case robot::hardwareSocket::RequestCodes::BUTTONS_PUSHED:
                m_response.head = robot::hardwareSocket::ResponseCodes::OK;
                m_response.body.buttons.button1 = hardwareInterface::buttons::getButtons(0);
                m_response.body.buttons.button2 = hardwareInterface::buttons::getButtons(1);
				m_response.body.buttons.button3 = hardwareInterface::buttons::getButtons(2);
                startSendResponse(sizeof(m_response.body.buttons));
            break;
		
			
			
            
            default:
               throw std::runtime_error("Unhandled request!");
        }
    }
}


void Session::startSendResponse(std::size_t bodySize)
{
    m_socket.async_send(boost::asio::buffer(&m_response, sizeof(m_response.head) + bodySize),
        boost::bind(&Session::onResponseSent, this, boost::asio::placeholders::error));
}

void Session::onResponseSent(const boost::system::error_code& error)
{
    if (error) {
        // todo: log
        m_controlSocket.dropSession(this);
    } else {
        startRecvRequestHead();
    }
}


void Session::startSendFailureCode()
{
    m_socket.async_send(boost::asio::buffer(&m_response.head, sizeof(m_response.head)),
        boost::bind(&Session::onResponseFailureCodeSent, this, boost::asio::placeholders::error));
}

void Session::onResponseFailureCodeSent(const boost::system::error_code& error)
{
    m_controlSocket.dropSession(this);    
}

