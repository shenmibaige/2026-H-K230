#ifndef __PID_H
#define __PID_H

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

    /* Low-pass filtered error derivative: larger DerivativeAlpha = more smoothing */
    float ErrorDerivativeFiltered;
    float DerivativeAlpha;

    /* Maximum output change per frame; 0 means unlimited */
    float OutputDeltaMax;

    float OutMax;
    float OutMin;

    /* 1 = disable integration while moving, 0 = allow integration */
    unsigned char IntegralHold;
} PID_t;

void PID_Update(PID_t *p);

#endif
