#ifndef MODE_INTERFACE_H
#define MODE_INTERFACE_H

#include <stdint.h>

typedef enum
{
    APP_MODE_Q2 = 0,
    APP_MODE_Q3,
    APP_MODE_Q4,
    APP_MODE_Q6,
    APP_MODE_COUNT
} AppModeId;

void ModeQ2_Init(void);
void ModeQ2_Start(void);
void ModeQ2_Tick1ms(void);
void ModeQ2_Loop(void);
uint8_t ModeQ2_IsDone(void);
void ModeQ2_Stop(void);

void ModeQ3_Init(void);
void ModeQ3_Start(void);
void ModeQ3_Tick1ms(void);
void ModeQ3_Loop(void);
uint8_t ModeQ3_IsDone(void);
void ModeQ3_Stop(void);

void ModeQ4_Init(void);
void ModeQ4_Start(void);
void ModeQ4_Tick1ms(void);
void ModeQ4_Loop(void);
uint8_t ModeQ4_IsDone(void);
void ModeQ4_Stop(void);

void ModeQ5_Init(void);
void ModeQ5_Start(void);
void ModeQ5_Tick1ms(void);
void ModeQ5_Loop(void);
uint8_t ModeQ5_IsDone(void);
void ModeQ5_Stop(void);

#endif
