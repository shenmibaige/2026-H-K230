#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

void Encoder_Init(void);
void Encoder_Reset(void);
int32_t Encoder_ReadDelta1(void);
int32_t Encoder_ReadDelta2(void);
int16_t Encoder_Get1(void);
int16_t Encoder_Get2(void);

#endif
