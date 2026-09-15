#include "mode_q2.h"

#include <stdint.h>

#include "app_common.h"
#include "curve_control_q2.h"
#include "curve_params_q2.h"
#include "encoder.h"
#include "huidu.h"
#include "motor.h"
#include "oled.h"

typedef struct
{
    Q2CurveControlOutput curve;
    int32_t encoder1_delta;
    int32_t encoder2_delta;
    uint32_t uptime_ms;
} ModeQ2Telemetry;

static Q2CurveControl g_curve;
static volatile ModeQ2Telemetry g_telemetry;
static volatile uint32_t g_uptime_ms;
static uint8_t g_control_divider;
static uint8_t g_start_requested;
static uint8_t g_started_once;
static uint32_t g_last_oled_ms;

static uint8_t ModeQ2_IsRunState(CurveRaceState state)
{
    return ((state == CURVE_STATE_STARTING) ||
            (state == CURVE_STATE_RUNNING) ||
            (state == CURVE_STATE_FINISH_ARMED))
               ? 1U
               : 0U;
}


static ModeQ2Telemetry ModeQ2_ReadTelemetry(void)
{
    ModeQ2Telemetry snapshot;
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

static void ModeQ2_PublishTelemetry(
    Q2CurveControlOutput output,
    int32_t encoder1_delta,
    int32_t encoder2_delta)
{
    g_telemetry.curve = output;
    g_telemetry.encoder1_delta = encoder1_delta;
    g_telemetry.encoder2_delta = encoder2_delta;
    g_telemetry.uptime_ms = g_uptime_ms;
}


static void ModeQ2_RenderOled(const ModeQ2Telemetry *snapshot)
{
    int32_t error_mm;

    if (snapshot == 0)
    {
        return;
    }

    error_mm =
        (int32_t)snapshot->curve.error_mm_q8 / CURVE_Q8_SCALE;

    OLED_ClearBuffer();
    OLED_DrawPrintf(0, 0, 16, "%s S%u",
                    Q2CurveControl_StateName(snapshot->curve.state),
                    (unsigned)g_run_seconds);
    OLED_DrawPrintf(0, 16, 16, "M=%02u F=%02u E=%d",
                    snapshot->curve.raw_mask,
                    snapshot->curve.filtered_mask,
                    error_mm);
    OLED_DrawPrintf(0, 32, 16, "U=%d A=%u G=%u",
                    snapshot->curve.turn_pwm,
                    snapshot->curve.approach_stage,
                    snapshot->curve.gap_hold);

    if (((snapshot->uptime_ms / 500U) & 1U) == 0U)
    {
        OLED_DrawPrintf(0, 48, 16, "L=%d R=%d",
                        snapshot->curve.left_pwm,
                        snapshot->curve.right_pwm);
    }
    else
    {
        OLED_DrawPrintf(0, 48, 16, "1=%d 2=%d",
                        snapshot->encoder1_delta,
                        snapshot->encoder2_delta);
    }
    OLED_Refresh();
}

void ModeQ2_Init(void)
{
    Q2CurveControl_Init(&g_curve);
    g_uptime_ms = 0U;
    g_control_divider = 0U;
    g_start_requested = 0U;
    g_started_once = 0U;
    g_last_oled_ms = UINT32_MAX;
    Encoder_Init();
    Encoder_Reset();
    Motor_StopSafe();
    ModeQ2_PublishTelemetry(
        Q2CurveControl_GetOutput(&g_curve),
        0L,
        0L);
}

void ModeQ2_Start(void)
{
    g_start_requested = 1U;
    Encoder_Reset();
}

void ModeQ2_Tick1ms(void)
{
    Q2CurveControlOutput output;
    CurveRaceState state_before_sample;

    g_uptime_ms++;
    huidu_get_value();
    state_before_sample =
        Q2CurveControl_GetOutput(&g_curve).state;
    Q2CurveControl_PushSample1ms(
        &g_curve,
        huidu_get_mask());
    output = Q2CurveControl_GetOutput(&g_curve);

    if ((state_before_sample == CURVE_STATE_FINISH_ARMED) &&
        (output.state == CURVE_STATE_STOPPED) &&
        (output.finish_trigger == CURVE_FINISH_TRIGGER_RAW_FAST))
    {
        Motor_StopSafe();
        g_control_divider = 0U;
        ModeQ2_PublishTelemetry(output, 0L, 0L);
        return;
    }

    if (g_start_requested != 0U)
    {
        if (Q2CurveControl_Start(&g_curve) != 0U)
        {
            g_start_requested = 0U;
            g_started_once = 1U;
            Encoder_Reset();
        }
    }

    g_control_divider++;
    if (g_control_divider >= CURVE_CONTROL_PERIOD_MS)
    {
        int32_t encoder1_delta;
        int32_t encoder2_delta;

        g_control_divider = 0U;
        output = Q2CurveControl_Step5ms(&g_curve);
        encoder1_delta = Encoder_ReadDelta1();
        encoder2_delta = Encoder_ReadDelta2();

        if (ModeQ2_IsRunState(output.state) != 0U)
        {
            Motor1_SetSpeed(output.right_pwm);
            Motor2_SetSpeed(output.left_pwm);
        }
        else
        {
            Motor_StopSafe();
        }

        ModeQ2_PublishTelemetry(
            output,
            encoder1_delta,
            encoder2_delta);
    }
}

void ModeQ2_Loop(void)
{
    ModeQ2Telemetry snapshot = ModeQ2_ReadTelemetry();

    if ((g_last_oled_ms == UINT32_MAX) ||
        ((snapshot.uptime_ms - g_last_oled_ms) >=
         CURVE_OLED_REFRESH_MS))
    {
        g_last_oled_ms = snapshot.uptime_ms;
        ModeQ2_RenderOled(&snapshot);
    }
}

uint8_t ModeQ2_IsDone(void)
{
    CurveRaceState state = Q2CurveControl_GetOutput(&g_curve).state;

    if (g_started_once == 0U)
    {
        return 0U;
    }
    return ((state == CURVE_STATE_STOPPED) ||
            (state == CURVE_STATE_FAULT))
               ? 1U
               : 0U;
}

void ModeQ2_Stop(void)
{
    Q2CurveControl_StopImmediate(
        &g_curve,
        CURVE_FAULT_NONE);
    Motor_StopSafe();
    Encoder_Reset();
    g_start_requested = 0U;
    ModeQ2_PublishTelemetry(
        Q2CurveControl_GetOutput(&g_curve),
        0L,
        0L);
}
