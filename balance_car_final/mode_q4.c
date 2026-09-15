#include "mode_q4.h"

#include <stdint.h>

#include "app_common.h"
#include "curve_control_v4.h"
#include "curve_params_v4.h"
#include "encoder.h"
#include "huidu.h"
#include "motor.h"
#include "oled.h"
#include "pid.h"
#include "servo.h"
#include "uart.h"

#define MODE_Q4_MOTOR_STOP_MS     8000U
#define MODE_Q4_SERVO_CENTER      85.0f
#define MODE_Q4_SERVO_MAX_OUT     65.0f
#define MODE_Q4_PID_AGGRESSIVE    5.0f
#define MODE_Q4_DX_INVALID        (-999)
#define MODE_Q4_DX_MAX_ABS        450
#define MODE_Q4_HOLD_LIMIT_PX     60.0f

#define MODE_Q4_S1_MAX_PX         10.0f
#define MODE_Q4_S2_MAX_PX         30.0f
#define MODE_Q4_S3_MAX_PX         60.0f
#define MODE_Q4_S4_MAX_PX         100.0f
#define MODE_Q4_S5_MAX_PX         160.0f
#define MODE_Q4_S6_MAX_PX         9999.0f

#define MODE_Q4_S1_KP             0.020f
#define MODE_Q4_S1_KI             0.0012f
#define MODE_Q4_S1_KD             1.4f
#define MODE_Q4_S1_OUT            8.0f
#define MODE_Q4_S1_DELTA          1.0f
#define MODE_Q4_S2_KP             0.028f
#define MODE_Q4_S2_KI             0.0008f
#define MODE_Q4_S2_KD             1.3f
#define MODE_Q4_S2_OUT            12.0f
#define MODE_Q4_S2_DELTA          1.5f
#define MODE_Q4_S3_KP             0.035f
#define MODE_Q4_S3_KI             0.0005f
#define MODE_Q4_S3_KD             1.2f
#define MODE_Q4_S3_OUT            20.0f
#define MODE_Q4_S3_DELTA          2.5f
#define MODE_Q4_S4_KP             0.045f
#define MODE_Q4_S4_KI             0.0003f
#define MODE_Q4_S4_KD             1.1f
#define MODE_Q4_S4_OUT            35.0f
#define MODE_Q4_S4_DELTA          4.0f
#define MODE_Q4_S5_KP             0.055f
#define MODE_Q4_S5_KI             0.0f
#define MODE_Q4_S5_KD             1.0f
#define MODE_Q4_S5_OUT            45.0f
#define MODE_Q4_S5_DELTA          6.0f
#define MODE_Q4_S6_KP             0.070f
#define MODE_Q4_S6_KI             0.0f
#define MODE_Q4_S6_KD             1.0f
#define MODE_Q4_S6_OUT            55.0f
#define MODE_Q4_S6_DELTA          8.0f

#define MODE_Q4_DERIVATIVE_ALPHA  0.40f
#define MODE_Q4_OUT_DEADBAND      0.08f
#define MODE_Q4_DX_DEADBAND       2.0f
#define MODE_Q4_SPEED_DEADBAND    0.50f
#define MODE_Q4_INTEGRAL_ERR_MAX  60.0f
#define MODE_Q4_INTEGRAL_SPEED    0.30f
#define MODE_Q4_STARTUP_MS        3000U
#define MODE_Q4_FADE_MS           500U
#define MODE_Q4_VEL_STAGE_GAIN    10.0f
#define MODE_Q4_STARTUP_ALPHA     0.18f

typedef struct
{
    CurveControlV4Output curve;
    int32_t encoder1_delta;
    int32_t encoder2_delta;
    uint32_t uptime_ms;
} ModeQ4Telemetry;

typedef struct
{
    float MaxAbsDx;
    float Kp;
    float Ki;
    float Kd;
    float OutMax;
    float OutDelta;
} ModeQ4PidStage;

static CurveControlV4 g_curve;
static volatile ModeQ4Telemetry g_telemetry;
static volatile uint32_t g_uptime_ms;
static uint8_t g_control_divider;
static uint8_t g_curve_start_pending;
static uint8_t g_started_once;
static uint32_t g_ball_start_ms;
static uint32_t g_last_oled_ms;

static volatile int Ball_Dx;
static volatile uint32_t UART_RxCount;
static volatile float Ball_Servo_Angle;
static volatile uint8_t Ball_Control_Enable;
static volatile uint8_t Ball_Within1cm;
static PID_t Ball_PID;
static uint8_t g_ball_valid_prev;

static const ModeQ4PidStage Ball_PidStages[6] = {
    {MODE_Q4_S1_MAX_PX, MODE_Q4_S1_KP, MODE_Q4_S1_KI, MODE_Q4_S1_KD, MODE_Q4_S1_OUT, MODE_Q4_S1_DELTA},
    {MODE_Q4_S2_MAX_PX, MODE_Q4_S2_KP, MODE_Q4_S2_KI, MODE_Q4_S2_KD, MODE_Q4_S2_OUT, MODE_Q4_S2_DELTA},
    {MODE_Q4_S3_MAX_PX, MODE_Q4_S3_KP, MODE_Q4_S3_KI, MODE_Q4_S3_KD, MODE_Q4_S3_OUT, MODE_Q4_S3_DELTA},
    {MODE_Q4_S4_MAX_PX, MODE_Q4_S4_KP, MODE_Q4_S4_KI, MODE_Q4_S4_KD, MODE_Q4_S4_OUT, MODE_Q4_S4_DELTA},
    {MODE_Q4_S5_MAX_PX, MODE_Q4_S5_KP, MODE_Q4_S5_KI, MODE_Q4_S5_KD, MODE_Q4_S5_OUT, MODE_Q4_S5_DELTA},
    {MODE_Q4_S6_MAX_PX, MODE_Q4_S6_KP, MODE_Q4_S6_KI, MODE_Q4_S6_KD, MODE_Q4_S6_OUT, MODE_Q4_S6_DELTA}
};

static float ModeQ4_AbsF(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float ModeQ4_StartupProgressFade(void)
{
    uint32_t elapsed;

    if (g_ball_start_ms == 0U)
    {
        return 0.0f;
    }

    elapsed = g_uptime_ms - g_ball_start_ms;
    if (elapsed >= MODE_Q4_STARTUP_MS + MODE_Q4_FADE_MS)
    {
        return 0.0f;
    }
    if (elapsed < MODE_Q4_STARTUP_MS)
    {
        return 1.0f;
    }

    return (float)(MODE_Q4_STARTUP_MS + MODE_Q4_FADE_MS - elapsed)
         / (float)MODE_Q4_FADE_MS;
}

static float ModeQ4_StartupStageGain(void)
{
    return MODE_Q4_VEL_STAGE_GAIN * ModeQ4_StartupProgressFade();
}

static void ModeQ4_UpdateDerivativeAlpha(void)
{
    float startupFactor = ModeQ4_StartupProgressFade();

    Ball_PID.DerivativeAlpha = MODE_Q4_STARTUP_ALPHA
        + (MODE_Q4_DERIVATIVE_ALPHA - MODE_Q4_STARTUP_ALPHA)
          * (1.0f - startupFactor);
}

static void ModeQ4_UpdatePidGains(
    float absDx,
    float absDeriv,
    float velocityStageGain)
{
    uint8_t i;
    float outMax;
    float effectiveDx;

    effectiveDx = absDx + absDeriv * velocityStageGain;
    if (effectiveDx > MODE_Q4_S6_MAX_PX)
    {
        effectiveDx = MODE_Q4_S6_MAX_PX;
    }

    for (i = 0U; i < 6U; i++)
    {
        if (effectiveDx <= Ball_PidStages[i].MaxAbsDx)
        {
            Ball_PID.Kp = Ball_PidStages[i].Kp * MODE_Q4_PID_AGGRESSIVE;
            Ball_PID.Ki = Ball_PidStages[i].Ki * MODE_Q4_PID_AGGRESSIVE;
            Ball_PID.Kd = Ball_PidStages[i].Kd * MODE_Q4_PID_AGGRESSIVE;
            Ball_PID.OutputDeltaMax = Ball_PidStages[i].OutDelta * MODE_Q4_PID_AGGRESSIVE;

            outMax = Ball_PidStages[i].OutMax * MODE_Q4_PID_AGGRESSIVE;
            if (outMax > MODE_Q4_SERVO_MAX_OUT)
            {
                outMax = MODE_Q4_SERVO_MAX_OUT;
            }
            Ball_PID.OutMax = outMax;
            Ball_PID.OutMin = -outMax;
            return;
        }
    }

    Ball_PID.Kp = MODE_Q4_S6_KP * MODE_Q4_PID_AGGRESSIVE;
    Ball_PID.Ki = MODE_Q4_S6_KI * MODE_Q4_PID_AGGRESSIVE;
    Ball_PID.Kd = MODE_Q4_S6_KD * MODE_Q4_PID_AGGRESSIVE;
    Ball_PID.OutputDeltaMax = MODE_Q4_S6_DELTA * MODE_Q4_PID_AGGRESSIVE;
    Ball_PID.OutMax = MODE_Q4_SERVO_MAX_OUT;
    Ball_PID.OutMin = -MODE_Q4_SERVO_MAX_OUT;
}

static int ModeQ4_ParseDx(const volatile char *packet)
{
    int sign = 1;
    int value = 0;
    const volatile char *p = packet;

    if (*p == '-')
    {
        sign = -1;
        p++;
    }
    else if (*p == '+')
    {
        p++;
    }

    if ((*p < '0') || (*p > '9'))
    {
        return MODE_Q4_DX_INVALID;
    }

    while ((*p >= '0') && (*p <= '9'))
    {
        int digit = *p - '0';
        if (value > (10000 - digit) / 10)
        {
            return MODE_Q4_DX_INVALID;
        }
        value = value * 10 + digit;
        p++;
    }

    if (*p != '\0')
    {
        return MODE_Q4_DX_INVALID;
    }

    return sign * value;
}

static void Ball_Control_Tick(void)
{
    float absDx;
    float absDeriv;
    int parsedDx;
    float errorNow;
    float filteredDeriv;
    float rawDeriv;
    float stageGain;

    if (UART_RxFlag != 1U)
    {
        return;
    }

    parsedDx = ModeQ4_ParseDx(UART_RxPacket);
    UART_RxFlag = 0U;
    Ball_Dx = parsedDx;
    UART_RxCount++;

    if (Ball_Control_Enable == 0U)
    {
        g_ball_valid_prev = 0U;
        return;
    }

    if ((parsedDx < -MODE_Q4_DX_MAX_ABS) ||
        (parsedDx > MODE_Q4_DX_MAX_ABS))
    {
        g_ball_valid_prev = 0U;
        return;
    }

    absDx = (float)(parsedDx < 0 ? -parsedDx : parsedDx);
    Ball_Within1cm = (absDx <= MODE_Q4_HOLD_LIMIT_PX) ? 1U : 0U;

    if (g_ball_valid_prev == 0U)
    {
        Ball_PID.ErrorInt = 0.0f;
        Ball_PID.Out = 0.0f;
        Ball_PID.ErrorDerivativeFiltered = 0.0f;
        Ball_PID.Actual = (float)parsedDx;
        Ball_PID.Error0 = Ball_PID.Target - Ball_PID.Actual;
        Ball_PID.Error1 = Ball_PID.Error0;
        g_ball_valid_prev = 1U;
    }
    else
    {
        Ball_PID.Actual = (float)parsedDx;
    }

    ModeQ4_UpdateDerivativeAlpha();
    stageGain = ModeQ4_StartupStageGain();

    errorNow = Ball_PID.Target - Ball_PID.Actual;
    rawDeriv = errorNow - Ball_PID.Error0;
    filteredDeriv = Ball_PID.DerivativeAlpha * Ball_PID.ErrorDerivativeFiltered
                  + (1.0f - Ball_PID.DerivativeAlpha) * rawDeriv;
    absDeriv = ModeQ4_AbsF(filteredDeriv);

    ModeQ4_UpdatePidGains(absDx, absDeriv, stageGain);

    Ball_PID.IntegralHold =
        (absDx > MODE_Q4_INTEGRAL_ERR_MAX) ||
        (absDeriv > MODE_Q4_INTEGRAL_SPEED);

    PID_Update(&Ball_PID);

    if ((absDx <= MODE_Q4_DX_DEADBAND) &&
        (ModeQ4_AbsF(Ball_PID.ErrorDerivativeFiltered) <=
         MODE_Q4_SPEED_DEADBAND))
    {
        Ball_PID.Out = 0.0f;
        Ball_PID.ErrorInt = 0.0f;
    }
    else if (ModeQ4_AbsF(Ball_PID.Out) < MODE_Q4_OUT_DEADBAND)
    {
        Ball_PID.Out = 0.0f;
    }

    Ball_Servo_Angle = MODE_Q4_SERVO_CENTER - Ball_PID.Out;
    Servo_SetAngle1(Ball_Servo_Angle);
}

static uint8_t ModeQ4_IsRunState(CurveRaceState state)
{
    return ((state == CURVE_STATE_STARTING) ||
            (state == CURVE_STATE_RUNNING) ||
            (state == CURVE_STATE_FINISH_ARMED))
               ? 1U
               : 0U;
}

static ModeQ4Telemetry ModeQ4_ReadTelemetry(void)
{
    ModeQ4Telemetry snapshot;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    snapshot.curve = g_telemetry.curve;
    snapshot.encoder1_delta = g_telemetry.encoder1_delta;
    snapshot.encoder2_delta = g_telemetry.encoder2_delta;
    snapshot.uptime_ms = g_telemetry.uptime_ms;
    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }
    return snapshot;
}

static void ModeQ4_PublishTelemetry(
    CurveControlV4Output output,
    int32_t encoder1_delta,
    int32_t encoder2_delta)
{
    g_telemetry.curve = output;
    g_telemetry.encoder1_delta = encoder1_delta;
    g_telemetry.encoder2_delta = encoder2_delta;
    g_telemetry.uptime_ms = g_uptime_ms;
}

static void ModeQ4_RenderOled(const ModeQ4Telemetry *snapshot)
{
    int32_t error_mm;
    const char *ball_status;

    if (snapshot == 0)
    {
        return;
    }

    error_mm =
        (int32_t)snapshot->curve.error_mm_q8 / CURVE_Q8_SCALE;

    if (Ball_Control_Enable == 0U)
    {
        ball_status = "STOP";
    }
    else if (Ball_Within1cm != 0U)
    {
        ball_status = "OK";
    }
    else
    {
        ball_status = "RUN";
    }

    OLED_ClearBuffer();
    OLED_DrawPrintf(0, 0, 16, "%s S%u",
                    CurveControlV4_StateName(snapshot->curve.state),
                    (unsigned)g_run_seconds);
    OLED_DrawPrintf(0, 16, 16, "M=%02u F=%02u E=%d",
                    snapshot->curve.raw_mask,
                    snapshot->curve.filtered_mask,
                    error_mm);
    OLED_DrawPrintf(0, 32, 16, "U=%d D=%4d",
                    snapshot->curve.turn_pwm,
                    Ball_Dx);
    OLED_DrawPrintf(0, 48, 16, "L=%d R=%d %s",
                    snapshot->curve.left_pwm,
                    snapshot->curve.right_pwm,
                    ball_status);
    OLED_Refresh();
}

void ModeQ4_Init(void)
{
    Ball_PID.Target = 0.0f;
    Ball_PID.Actual = 0.0f;
    Ball_PID.Out = 0.0f;
    Ball_PID.Error0 = 0.0f;
    Ball_PID.Error1 = 0.0f;
    Ball_PID.ErrorInt = 0.0f;
    Ball_PID.ErrorDerivativeFiltered = 0.0f;
    Ball_PID.DerivativeAlpha = MODE_Q4_DERIVATIVE_ALPHA;
    Ball_PID.IntegralHold = 0U;
    ModeQ4_UpdatePidGains(0.0f, 0.0f, 0.0f);

    CurveControlV4_Init(&g_curve);
    g_uptime_ms = 0U;
    g_control_divider = 0U;
    g_curve_start_pending = 0U;
    g_started_once = 0U;
    g_ball_start_ms = 0U;
    g_ball_valid_prev = 0U;
    g_last_oled_ms = UINT32_MAX;

    Encoder_Init();
    Encoder_Reset();
    Motor_StopSafe();
    Servo_SetAngle1(MODE_Q4_SERVO_CENTER);
    Ball_Control_Enable = 0U;
    Ball_Servo_Angle = MODE_Q4_SERVO_CENTER;
    Ball_Within1cm = 0U;
    Ball_Dx = 0;
    UART_RxCount = 0U;

    ModeQ4_PublishTelemetry(
        CurveControlV4_GetOutput(&g_curve),
        0L,
        0L);
}

void ModeQ4_Start(void)
{
    Servo_SetAngle1(MODE_Q4_SERVO_CENTER);
    if (Ball_Control_Enable == 0U)
    {
        g_ball_start_ms = g_uptime_ms;
    }
    Ball_Control_Enable = 1U;
    g_curve_start_pending = 1U;
    g_started_once = 1U;
    Encoder_Reset();
}

void ModeQ4_Tick1ms(void)
{
    CurveControlV4Output output;
    CurveRaceState state_before_sample;

    g_uptime_ms++;
    huidu_get_value();
    Ball_Control_Tick();

    state_before_sample =
        CurveControlV4_GetOutput(&g_curve).state;
    CurveControlV4_PushSample1ms(
        &g_curve,
        huidu_get_mask());
    output = CurveControlV4_GetOutput(&g_curve);

    if ((state_before_sample == CURVE_STATE_FINISH_ARMED) &&
        (output.state == CURVE_STATE_STOPPED) &&
        (output.finish_trigger == CURVE_FINISH_TRIGGER_RAW_FAST))
    {
        Motor_StopSafe();
        g_control_divider = 0U;
        ModeQ4_PublishTelemetry(output, 0L, 0L);
        return;
    }

    if ((ModeQ4_IsRunState(output.state) != 0U) &&
        (output.elapsed_ms >= MODE_Q4_MOTOR_STOP_MS))
    {
        CurveControlV4_StopImmediate(
            &g_curve,
            CURVE_FAULT_NONE);
        Motor_StopSafe();
        g_control_divider = 0U;
        g_curve_start_pending = 0U;
        ModeQ4_PublishTelemetry(
            CurveControlV4_GetOutput(&g_curve),
            0L,
            0L);
        return;
    }

    if (g_curve_start_pending != 0U)
    {
        if (CurveControlV4_Start(&g_curve) != 0U)
        {
            g_curve_start_pending = 0U;
            Encoder_Reset();
        }
        else if ((CurveControlV4_GetOutput(&g_curve).state != CURVE_STATE_WAIT) &&
                 (CurveControlV4_GetOutput(&g_curve).state != CURVE_STATE_STOPPED) &&
                 (CurveControlV4_GetOutput(&g_curve).state != CURVE_STATE_FAULT))
        {
            g_curve_start_pending = 0U;
        }
    }

    g_control_divider++;
    if (g_control_divider >= CURVE_CONTROL_PERIOD_MS)
    {
        int32_t encoder1_delta;
        int32_t encoder2_delta;

        g_control_divider = 0U;
        output = CurveControlV4_Step5ms(&g_curve);
        encoder1_delta = Encoder_ReadDelta1();
        encoder2_delta = Encoder_ReadDelta2();

        if (ModeQ4_IsRunState(output.state) != 0U)
        {
            Motor1_SetSpeed(output.right_pwm);
            Motor2_SetSpeed(output.left_pwm);
        }
        else
        {
            Motor_StopSafe();
        }

        ModeQ4_PublishTelemetry(
            output,
            encoder1_delta,
            encoder2_delta);
    }
}

void ModeQ4_Loop(void)
{
    ModeQ4Telemetry snapshot = ModeQ4_ReadTelemetry();

    if ((g_last_oled_ms == UINT32_MAX) ||
        ((snapshot.uptime_ms - g_last_oled_ms) >=
         CURVE_OLED_REFRESH_MS))
    {
        g_last_oled_ms = snapshot.uptime_ms;
        ModeQ4_RenderOled(&snapshot);
    }
}

uint8_t ModeQ4_IsDone(void)
{
    CurveRaceState state = CurveControlV4_GetOutput(&g_curve).state;

    if (g_started_once == 0U)
    {
        return 0U;
    }
    return ((state == CURVE_STATE_STOPPED) ||
            (state == CURVE_STATE_FAULT))
               ? 1U
               : 0U;
}

void ModeQ4_Stop(void)
{
    CurveControlV4_StopImmediate(
        &g_curve,
        CURVE_FAULT_NONE);
    Motor_StopSafe();
    Ball_Control_Enable = 0U;
    Ball_PID.Out = 0.0f;
    Ball_PID.ErrorInt = 0.0f;
    Ball_PID.ErrorDerivativeFiltered = 0.0f;
    Ball_Servo_Angle = MODE_Q4_SERVO_CENTER;
    Servo_SetAngle1(Ball_Servo_Angle);
    g_curve_start_pending = 0U;
    g_ball_start_ms = 0U;
    g_ball_valid_prev = 0U;
    Encoder_Reset();
    ModeQ4_PublishTelemetry(
        CurveControlV4_GetOutput(&g_curve),
        0L,
        0L);
}
