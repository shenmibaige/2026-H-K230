#include "curve_pid.h"

void Curve_PID_Reset(Curve_PID_t *p)
{
    if (p == 0)
    {
        return;
    }

    p->Target = 0.0f;
    p->Actual = 0.0f;
    p->Out = 0.0f;
    p->Error0 = 0.0f;
    p->Error1 = 0.0f;
    p->ErrorInt = 0.0f;
}

void Curve_PID_SetTarget(Curve_PID_t *p, float target)
{
    if ((p == 0) || (p->Target == target))
    {
        return;
    }

    p->Target = target;
    p->ErrorInt = 0.0f;
    p->Error0 = target - p->Actual;
    p->Error1 = p->Error0;
}

void Curve_PID_SetActualMagnitude(Curve_PID_t *p, int16_t encoder_delta)
{
    if (p == 0)
    {
        return;
    }

    /* Both motors use mirrored electrical forward directions. The speed
     * controller controls wheel speed magnitude, not signed encoder count. */
    if (encoder_delta < 0)
    {
        p->Actual = -(float)encoder_delta;
    }
    else
    {
        p->Actual = (float)encoder_delta;
    }
}

/**
 * PID calculation for the line-tracking controller.
 */
void Curve_PID_Update(Curve_PID_t *p)
{
    float previousOut;

    if (p == 0)
    {
        return;
    }

    previousOut = p->Out;

    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    if (p->Ki != 0)
    {
        p->ErrorInt += p->Error0;

        if (p->ErrorIntMax > p->ErrorIntMin)
        {
            if (p->ErrorInt > p->ErrorIntMax)
            {
                p->ErrorInt = p->ErrorIntMax;
            }
            if (p->ErrorInt < p->ErrorIntMin)
            {
                p->ErrorInt = p->ErrorIntMin;
            }
        }
        else
        {
            float intLimit = p->OutMax / p->Ki;
            if (intLimit < 0.0f)
            {
                intLimit = -intLimit;
            }
            if (p->ErrorInt > intLimit)  p->ErrorInt = intLimit;
            if (p->ErrorInt < -intLimit) p->ErrorInt = -intLimit;
        }
    }
    else
    {
        p->ErrorInt = 0;
    }

    p->Out = p->Kp * p->Error0
           + p->Ki * p->ErrorInt
           + p->Kd * (p->Error0 - p->Error1);

    if (p->Out > p->OutMax) { p->Out = p->OutMax; }
    if (p->Out < p->OutMin) { p->Out = p->OutMin; }

    if ((p->OutRiseMax > 0.0f) &&
        (p->Out > (previousOut + p->OutRiseMax)))
    {
        p->Out = previousOut + p->OutRiseMax;
    }
    if ((p->OutFallMax > 0.0f) &&
        (p->Out < (previousOut - p->OutFallMax)))
    {
        p->Out = previousOut - p->OutFallMax;
    }
}
