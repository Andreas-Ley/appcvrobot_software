#ifndef PROTOCOLL_H
#define PROTOCOLL_H


#define I2C_ADDRESS           (0x10)

#define REGISTER_STATUS       		(0x00)
#define REGISTER_ENABLE_MOTOR  		(0x01)
#define REGISTER_STEPS_MOVED  		(0x02)
#define REGISTER_SET_TARGET_SPEED (0x04)
#define REGISTER_SET_LED       		(0x05)
#define REGISTER_CPU_USAGE        (0x06)
#define REGISTER_BATTERY_DRAW     (0x10)
#define REGISTER_CELL_VOLTAGE_1   (0x11)
#define REGISTER_CELL_VOLTAGE_2   (0x12)
#define REGISTER_CELL_VOLTAGE_3   (0x13)
#define REGISTER_INITIATE_SHUTDOWN         (0x20)
#define REGISTER_BUTTONS                   (0x21)
#define BUTTON_POWER          (0x01)
#define BUTTON_EXECUTE        (0x02)
#define BUTTON_STOP           (0x04)


#endif
