#ifndef __PID_Q2_H
#define __PID_Q2_H

#include <stdint.h>

typedef struct {
	float Target;
	float Actual;
	float Out;
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;
	float Error1;
	float ErrorInt;
	float ErrorIntMax;
	float ErrorIntMin;

	float OutMax;
	float OutMin;

	float OutRiseMax;
	float OutFallMax;
} Q2_PID_t;

void Q2_PID_Reset(Q2_PID_t *p);
void Q2_PID_SetTarget(Q2_PID_t *p, float target);
void Q2_PID_SetActualMagnitude(Q2_PID_t *p, int16_t encoder_delta);
void Q2_PID_Update(Q2_PID_t *p);

#endif
