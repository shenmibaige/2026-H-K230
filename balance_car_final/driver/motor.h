#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

void Motor1_SetSpeed(int16_t Speed);
void Motor2_SetSpeed(int16_t Speed);
void Motor1_Brake(void);
void Motor2_Brake(void);
void Motor_StopSafe(void);

#endif
