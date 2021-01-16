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

#ifndef CONTROLSOCKETPROTOCOL_H
#define CONTROLSOCKETPROTOCOL_H

#include <string>

#include <cstdint>

namespace robot {   
namespace hardwareSocket {
	
enum class RequestCodes : std::uint8_t 
{ 
    // status stuff
    BATTERY_CELL_VOLTAGES   = 0x00, /// Request battery cell voltages
    CURRENT_DRAW            = 0x01, /// Request current draw
    CONTOLLER_USAGE         = 0x02, /// Request cpu usage of the micro controller
    // drive stuff
    DRIVE_ACQUIRE           = 0x10, /// Acquire sole control over the drive
    DRIVE_RELEASE           = 0x11, /// Release sole control over the drive
    DRIVE_SET_SPEED         = 0x12, /// Set speeds of drive
    DRIVE_GET_STEPS         = 0x13, /// Get drive steps taken and reset step counters
    DRIVE_ENABLE            = 0x14, /// Enable motors
    DRIVE_DISABLE           = 0x15, /// Disable motors
    // LCD stuff
    LCD_SET_TEXT            = 0x20, /// Set two lines of text on LCD and turns backlight on. Can be done without previous acquistion of lock.
    LCD_ACQUIRE             = 0x21, /// Acquire sole control over the LCD for fine-grained control
    LCD_RELEASE             = 0x22, /// Release sole control over the LCD for fine-grained control
    // button stuff
	BUTTONS_PUSHED			= 0x30, 

    // addon stuff
};
    
struct RequestBodyDriveSetSpeed {
    float speedLeft;
    float speedRight;
} __attribute__((packed));

struct RequestBodyLCDSetText {
    enum {
        LINE_LENGTH = 20,
        NUM_LINES = 2,
        LINE_BUFFER_SIZE = LINE_LENGTH+1
    };
    char lines[NUM_LINES][LINE_BUFFER_SIZE];
    
    void setLine(unsigned line, const std::string &src);
} __attribute__((packed));

union AllRequestBodies {
    RequestBodyDriveSetSpeed driveSetSpeed;
    RequestBodyLCDSetText LCDSetText;
};

struct Request {
    RequestCodes head;
    AllRequestBodies body;
} __attribute__((packed));


enum class ResponseCodes : std::uint8_t 
{ 
    OK                     = 0x00, /// Whatever it was, it was successfull
    UNSPECIFIED_ERROR      = 0x01, /// Something unspecified went wrong
    UNKNOWN_REQUEST        = 0x02, /// The request code was not understood, indicating a protocol error/mismatch
    INVALID_VALUE          = 0x03, /// Parameters were invalid our outside the allowed range
    
    DRIVE_ALREADY_ACQUIRED = 0x11, /// Acquisition of the lock on the drive failed because it was already acquired by a different process
    DRIVE_WAS_NOT_ACQUIRED = 0x12, /// Releasing of the lock on the drive failed because it was no acquired in the first place

    LCD_ALREADY_ACQUIRED   = 0x21, /// Acquisition of the lock on the LCD or setting LCD text failed because it was already acquired by a different process
    LCD_WAS_NOT_ACQUIRED   = 0x22, /// Releasing of the lock on the LCD failed because it was no acquired in the first place
};


struct ResponseBodyCellVoltages {
    enum {
        NUM_CELLS = 3
    };
    float voltages[NUM_CELLS];
} __attribute__((packed));

struct ResponseBodyDriveGetSteps {
    int stepsLeft;
    int stepsRight;
} __attribute__((packed));

struct ResponseBodyButtons {
    bool button1;
	bool button2;
	bool button3;
	
} __attribute__((packed));


struct ResponseBodyCpu {
    float cpu;
	
	
} __attribute__((packed));


struct ResponseBodyCurrent {
    float current;
	
	
} __attribute__((packed));

union AllResponseBodies {
    ResponseBodyCellVoltages cellVoltages;
    ResponseBodyDriveGetSteps driveGetSteps; 
	ResponseBodyButtons buttons;
	ResponseBodyCpu cpu;
	ResponseBodyCurrent current;
};

struct Response {
    ResponseCodes head;
    AllResponseBodies body;
} __attribute__((packed));


}
}

#endif // CONTROLSOCKETPROTOCOL_H
