#ifndef __CURVE_PID_H
#define __CURVE_PID_H

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
} Curve_PID_t;

void Curve_PID_Reset(Curve_PID_t *p);
void Curve_PID_SetTarget(Curve_PID_t *p, float target);
void Curve_PID_SetActualMagnitude(Curve_PID_t *p, int16_t encoder_delta);
void Curve_PID_Update(Curve_PID_t *p);

#endif
