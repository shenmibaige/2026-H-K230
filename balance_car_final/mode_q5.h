#ifndef MODE_Q5_H
#define MODE_Q5_H

#include <stdint.h>

void ModeQ5_Init(void);
void ModeQ5_Start(void);
void ModeQ5_Tick1ms(void);
void ModeQ5_Loop(void);
uint8_t ModeQ5_IsDone(void);
void ModeQ5_Stop(void);

#endif
