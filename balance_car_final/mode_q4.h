#ifndef MODE_Q4_H
#define MODE_Q4_H

#include <stdint.h>

void ModeQ4_Init(void);
void ModeQ4_Start(void);
void ModeQ4_Tick1ms(void);
void ModeQ4_Loop(void);
uint8_t ModeQ4_IsDone(void);
void ModeQ4_Stop(void);

#endif
