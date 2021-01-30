#include <Wire.h>

#include "protocoll.h"

#include "MotorControl.h"


#define PIN_SWITCH_SENSE  (A7)
#define PIN_HOLD_POWER  (13)
#define PIN_BUTTON_EXECUTE  (0)
#define PIN_BUTTON_STOP    (2)

bool powerButtonReleased = false;
bool initiateShutdown = false;

uint8_t buttonsPressed = 0;

void receiveEvent(int bytes);
void requestEvent();


void setup() {
    //Serial.begin(9600);
  
    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(receiveEvent); // register event
    Wire.onRequest(requestEvent); // register event

    motors::init();
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(PIN_HOLD_POWER, OUTPUT);
    digitalWrite(PIN_HOLD_POWER, HIGH);

    pinMode(PIN_BUTTON_EXECUTE, INPUT_PULLUP);
    pinMode(PIN_BUTTON_STOP, INPUT_PULLUP);
}

unsigned char cpuUsage = 0;
unsigned short cellVoltage1 = 0;
unsigned short cellVoltage2 = 0;
unsigned short cellVoltage3 = 0;
unsigned short batteryDraw = 0;

unsigned short lowVoltageCounter = 0;

unsigned analogReadCounter = 0;
  unsigned short tmp_cellVoltage1 = 0;
  unsigned short tmp_cellVoltage2 = 0;
  unsigned short tmp_cellVoltage3 = 0;
  unsigned short tmp_batteryDraw = 0;

void loop() {
  unsigned start = micros();
  delayMicroseconds(16383);
  unsigned end = micros();
  cpuUsage = 255 - (16383 / ((end-start)/255));

  tmp_cellVoltage1 += analogRead(A2);
  tmp_cellVoltage2 += analogRead(A3);
  tmp_cellVoltage3 += analogRead(A1);
  tmp_batteryDraw += analogRead(A0);
  analogReadCounter++;

  if (analogReadCounter == 8) {
    noInterrupts();
    cellVoltage1 = tmp_cellVoltage1 / 8;
    cellVoltage2 = tmp_cellVoltage2 / 8;
    cellVoltage3 = tmp_cellVoltage3 / 8;
    batteryDraw = tmp_batteryDraw / 8;
    interrupts();
    tmp_cellVoltage1 = 0;
    tmp_cellVoltage2 = 0;
    tmp_cellVoltage3 = 0;
    tmp_batteryDraw = 0;
    analogReadCounter = 0;
  
    bool emergencyShutdown = false;
    if (cellVoltage1 < 637) // 2.8V
        emergencyShutdown = true;
    if (cellVoltage2*2 - cellVoltage1 < 637) // 2.8V
        emergencyShutdown = true;
    if (cellVoltage3*3 - cellVoltage2*2 < 637) // 2.8V
        emergencyShutdown = true;
  
    if (emergencyShutdown)
        lowVoltageCounter++;
    else
        lowVoltageCounter = 0;

    if (lowVoltageCounter > 64)
        digitalWrite(PIN_HOLD_POWER, LOW);
  }
  unsigned short switchSense = analogRead(PIN_SWITCH_SENSE);

  if (switchSense < 300)
    powerButtonReleased = true;
  else if (switchSense > 600 && powerButtonReleased) {
    buttonsPressed |= BUTTON_POWER;
  }

  if (!digitalRead(PIN_BUTTON_EXECUTE))
    buttonsPressed |= BUTTON_EXECUTE;
  if (!digitalRead(PIN_BUTTON_STOP))
    buttonsPressed |= BUTTON_STOP;

  if (initiateShutdown) {
    delay(2000);
    digitalWrite(PIN_HOLD_POWER, LOW);
  }
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
    case REGISTER_INITIATE_SHUTDOWN: {
      initiateShutdown = true;
    } break;
		case REGISTER_ENABLE_MOTOR: {
      if (bytes != 2) break;
			motors::setEnabled(Wire.read());
		} break;
		case REGISTER_SET_TARGET_SPEED: {
      if (bytes != 5) break;
      uint16_t speedL = Wire.read() | (Wire.read() << 8);
      uint16_t speedR = Wire.read() | (Wire.read() << 8);
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
      if (bytes != 2) break;
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
    case REGISTER_CELL_VOLTAGES: {
      int32_t v = (int32_t)cellVoltage1 |
                (  ((int32_t)cellVoltage2) << 10 ) |
                (  ((int32_t)cellVoltage3) << 20 );
      Wire.write((uint8_t*)&v, 4);
    } break;
    case REGISTER_BUTTONS: {
      Wire.write(&buttonsPressed, 1);
      buttonsPressed = 0;
    } break;
	};
}
