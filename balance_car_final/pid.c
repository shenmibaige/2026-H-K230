#include "ti_msp_dl_config.h"
#include "pid.h"

/**
 * PID calculation. The caller updates Actual and calls this once per
 * received K230 frame, then applies Out to the servo angle.
 */
void PID_Update(PID_t *p)
{
    float ProportionalOut;
    float Derivative;
    float IntegralTerm;
    float OutCandidate;
    float PrevOut;

    PrevOut = p->Out;

    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;
    Derivative = p->Error0 - p->Error1;

    if (p->DerivativeAlpha > 0.0f && p->DerivativeAlpha < 1.0f)
    {
        p->ErrorDerivativeFiltered = p->DerivativeAlpha * p->ErrorDerivativeFiltered
                                   + (1.0f - p->DerivativeAlpha) * Derivative;
    }
    else
    {
        p->ErrorDerivativeFiltered = Derivative;
    }

    ProportionalOut = p->Kp * p->Error0
                    + p->Kd * p->ErrorDerivativeFiltered;

    if (p->Ki != 0.0f && p->IntegralHold == 0)
    {
        IntegralTerm = p->Ki * p->ErrorInt;
        OutCandidate = ProportionalOut + IntegralTerm;

        /* Anti-windup: stop integrating when the output is already
         * saturated in the direction of the current error. */
        if ((OutCandidate > p->OutMax && p->Error0 > 0) ||
            (OutCandidate < p->OutMin && p->Error0 < 0))
        {
            p->Out = (OutCandidate > p->OutMax) ? p->OutMax : p->OutMin;
        }
        else
        {
            p->ErrorInt += p->Error0;
            p->Out = ProportionalOut + p->Ki * p->ErrorInt;
            if (p->Out > p->OutMax) { p->Out = p->OutMax; }
            if (p->Out < p->OutMin) { p->Out = p->OutMin; }
        }
    }
    else
    {
        p->ErrorInt = 0.0f;
        p->Out = ProportionalOut;
        if (p->Out > p->OutMax) { p->Out = p->OutMax; }
        if (p->Out < p->OutMin) { p->Out = p->OutMin; }
    }

    /* Limit how fast the servo angle can change per frame. */
    if (p->OutputDeltaMax > 0.0f)
    {
        if (p->Out > PrevOut + p->OutputDeltaMax)
        {
            p->Out = PrevOut + p->OutputDeltaMax;
        }
        if (p->Out < PrevOut - p->OutputDeltaMax)
        {
            p->Out = PrevOut - p->OutputDeltaMax;
        }
        if (p->Out > p->OutMax) { p->Out = p->OutMax; }
        if (p->Out < p->OutMin) { p->Out = p->OutMin; }
    }
}
