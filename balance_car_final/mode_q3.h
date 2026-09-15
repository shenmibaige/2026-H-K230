#ifndef MODE_Q3_H
#define MODE_Q3_H

#include <stdint.h>

void ModeQ3_Init(void);
void ModeQ3_Start(void);
void ModeQ3_Tick1ms(void);
void ModeQ3_Loop(void);
uint8_t ModeQ3_IsDone(void);
void ModeQ3_Stop(void);

#endif
