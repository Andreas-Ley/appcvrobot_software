#include "MotorControl.h"

#include <Arduino.h>

#define PIN_DIR_R  (1)
#define PIN_STEP_R  (3)
#define PIN_DIR_L  (7)
#define PIN_STEP_L  (9)
#define PIN_ENABLE (8)

#define PIN_MS1_R (6)
#define PIN_MS2_R (5)
#define PIN_MS3_R (4)

#define PIN_MS1_L (10)
#define PIN_MS2_L (11)
#define PIN_MS3_L (12)

namespace motors {
    
static uint16_t stepDelays[2] = {};
static uint16_t counter[2] = {};
static int16_t stepsTaken[2] = {};
static char direction[2] = {1, 1};
static bool toggle[2] = {};
    
void init()
{
    pinMode(PIN_DIR_R, OUTPUT);
    pinMode(PIN_STEP_R, OUTPUT);
    pinMode(PIN_DIR_L, OUTPUT);
    pinMode(PIN_STEP_L, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);

    pinMode(PIN_MS1_R, OUTPUT);
    pinMode(PIN_MS2_R, OUTPUT);
    pinMode(PIN_MS3_R, OUTPUT);

    pinMode(PIN_MS1_L, OUTPUT);
    pinMode(PIN_MS2_L, OUTPUT);
    pinMode(PIN_MS3_L, OUTPUT);

    digitalWrite(PIN_MS1_R, LOW);
    digitalWrite(PIN_MS2_R, LOW);
    digitalWrite(PIN_MS3_R, LOW);
    digitalWrite(PIN_MS1_L, LOW);
    digitalWrite(PIN_MS2_L, LOW);
    digitalWrite(PIN_MS3_L, LOW);

    digitalWrite(PIN_ENABLE, LOW);


    noInterrupts();           // disable all interrupts

    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    
    OCR1A = 25;      // 8MHz / 64 / 25 = 5kHz
    
    TCCR1B |= (1 << WGM12);   // CTC mode
    TCCR1B |= (1 << CS11) | (1 << CS10);    // 64 prescaler 
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

    interrupts();             // enable all interrupts
}

template<unsigned pinStep, unsigned idx>
inline void operateMotor(uint16_t stepDelay) 
{
    counter[idx]++;
    if (counter[idx] > stepDelay) {
        counter[idx] = 0;
        toggle[idx] = !toggle[idx];
        digitalWrite(pinStep, toggle[idx]);

        if (toggle[idx]) {
          stepsTaken[idx] += direction[idx];
        }
    }
} 

ISR(TIMER1_COMPA_vect)
{
    uint16_t stepDelayL = stepDelays[0];
    uint16_t stepDelayR = stepDelays[1];

    if (stepDelayL != 0)
      operateMotor<PIN_STEP_L, 0>(stepDelayL);
    if (stepDelayR != 0)
      operateMotor<PIN_STEP_R, 1>(stepDelayR);
}

void setEnabled(bool enabled)
{
    digitalWrite(PIN_ENABLE, enabled?LOW:HIGH);
}

void setSpeed(int16_t left, int16_t right)
{
    TIMSK1 &= ~(1 << OCIE1A);
    stepDelays[0] = abs(left);
    stepDelays[1] = abs(right);
    TIMSK1 |= (1 << OCIE1A);

    direction[0] = (left > 0)?1:-1;
    direction[1] = (right > 0)?1:-1;
    digitalWrite(PIN_DIR_L, (left > 0)?HIGH:LOW);
    digitalWrite(PIN_DIR_R, (right > 0)?HIGH:LOW);
}

void fetchStepsTaken(uint16_t &left, uint16_t &right)
{
    TIMSK1 &= ~(1 << OCIE1A);
    left = stepsTaken[0];
    right = stepsTaken[1];
    stepsTaken[0] = 0;
    stepsTaken[1] = 0;
    TIMSK1 |= (1 << OCIE1A);
}
    
}
