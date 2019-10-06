#include <Wire.h>

#include "protocoll.h"

#include "MotorControl.h"



void receiveEvent(int bytes);
void requestEvent();


void setup() {
    //Serial.begin(9600);
  
    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(receiveEvent); // register event
    Wire.onRequest(requestEvent); // register event

    motors::init();
    pinMode(LED_BUILTIN, OUTPUT);
}

unsigned char cpuUsage = 0;

unsigned short cellVoltage1 = 0;
unsigned short cellVoltage2 = 0;
unsigned short cellVoltage3 = 0;
unsigned short batteryDraw = 0;

void loop() {
  unsigned start = micros();
  delayMicroseconds(16383);
  unsigned end = micros();
  cpuUsage = 255 - (16383 / ((end-start)/255));

  unsigned short tmp_cellVoltage1 = analogRead(A3);
  unsigned short tmp_cellVoltage2 = analogRead(A2);
  unsigned short tmp_cellVoltage3 = analogRead(A1);
  unsigned short tmp_batteryDraw = analogRead(A0);

  noInterrupts();
  cellVoltage1 = tmp_cellVoltage1;
  cellVoltage2 = tmp_cellVoltage2;
  cellVoltage3 = tmp_cellVoltage3;
  batteryDraw = tmp_batteryDraw;
  interrupts();

}


unsigned char lastAccessedRegister = 0;

uint16_t stepsCopy[2];

void receiveEvent(int bytes)
{
	lastAccessedRegister = Wire.read();
/*
Serial.print("recieve event ");	
Serial.print(lastAccessedRegister);
*/
	switch (lastAccessedRegister) {
		case REGISTER_ENABLE_MOTOR: {
      if (bytes != 1) break;
			motors::setEnabled(Wire.read());
		} break;
		case REGISTER_SET_TARGET_SPEED: {
      if (bytes != 5) break;
      int16_t speedL = Wire.read() | (Wire.read() << 8);
      int16_t speedR = Wire.read() | (Wire.read() << 8);
/*
Serial.print("REGISTER_SET_TARGET_SPEED ");
Serial.print(speedL);
Serial.print(" ");
Serial.print(speedR);
Serial.println("");
*/
			motors::setSpeed(
						speedL,
						speedR
			);
		} break;
		case REGISTER_SET_LED: {
      if (bytes != 1) break;
			digitalWrite(LED_BUILTIN, Wire.read()?HIGH:LOW);
		} break;
		case REGISTER_STEPS_MOVED: {
//Serial.println("REGISTER_STEPS_MOVED");
			motors::fetchStepsTaken(stepsCopy[0], stepsCopy[1]);
		} break;
	};
}

void requestEvent()
{
	switch (lastAccessedRegister) {
		case REGISTER_STEPS_MOVED: {
			Wire.write((uint8_t*)&stepsCopy, 2*sizeof(int16_t));
		} break;
		case REGISTER_STATUS: {
			unsigned char status = 0;
			Wire.write(&status, 1);
		} break;
    case REGISTER_CPU_USAGE: {
      Wire.write(&cpuUsage, 1);
    } break;
    case REGISTER_BATTERY_DRAW: {
      Wire.write((uint8_t*)&batteryDraw, 2);
    } break;
    case REGISTER_CELL_VOLTAGE_1: {
      Wire.write((uint8_t*)&cellVoltage1, 2);
    } break;
    case REGISTER_CELL_VOLTAGE_2: {
      Wire.write((uint8_t*)&cellVoltage2, 2);
    } break;
    case REGISTER_CELL_VOLTAGE_3: {
      Wire.write((uint8_t*)&cellVoltage3, 2);
    } break;
	};
}
