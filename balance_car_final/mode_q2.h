#ifndef MODE_Q2_H
#define MODE_Q2_H

#include <stdint.h>

void ModeQ2_Init(void);
void ModeQ2_Start(void);
void ModeQ2_Tick1ms(void);
void ModeQ2_Loop(void);
uint8_t ModeQ2_IsDone(void);
void ModeQ2_Stop(void);

#endif
