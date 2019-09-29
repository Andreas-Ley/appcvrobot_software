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

void loop() {
  unsigned start = micros();
  delayMicroseconds(16383);
  unsigned end = micros();
  cpuUsage = 255 - (16383 / ((end-start)/255));
  /*
  Serial.println("Cpu usage");
  Serial.println((int) cpuUsage);
  delay(1000);
  */
}


unsigned char lastAccessedRegister = 0;

uint16_t stepsCopy[2];

void receiveEvent(int bytes)
{
	lastAccessedRegister = Wire.read();
	
	switch (lastAccessedRegister) {
		case REGISTER_ENABLE_MOTOR: {
			motors::setEnabled(Wire.read());
		} break;
		case REGISTER_SET_TARGET_SPEED: {
			motors::setSpeed(
						(Wire.read() << 8) | Wire.read(),
						(Wire.read() << 8) | Wire.read()
			);
		} break;
		case REGISTER_SET_LED: {
			digitalWrite(LED_BUILTIN, Wire.read()?HIGH:LOW);
		} break;
		case REGISTER_STEPS_MOVED: {
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
	};
}
