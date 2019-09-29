#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

namespace motors {
    
void init();

void setEnabled(bool enabled);


// Must be called with interrupts disabled (e.g. from ISR)
void setSpeed(int16_t left, int16_t right);

// Must be called with interrupts disabled (e.g. from ISR)
void fetchStepsTaken(uint16_t &left, uint16_t &right);
    
}

#endif
