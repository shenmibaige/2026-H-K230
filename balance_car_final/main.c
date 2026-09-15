#include "ti_msp_dl_config.h"

#include <stdint.h>

#include "app_common.h"
#include "encoder.h"
#include "key.h"
#include "mode_interface.h"
#include "motor.h"
#include "oled.h"
#include "servo.h"
#include "uart.h"

volatile uint32_t system_ms = 0u;
volatile uint32_t g_run_seconds = 0u;

typedef enum
{
    APP_STATE_MENU = 0,
    APP_STATE_RUNNING
} AppState;

static volatile AppState g_app_state = APP_STATE_MENU;
static volatile AppModeId g_selected_mode = APP_MODE_Q2;
static volatile AppModeId g_active_mode = APP_MODE_Q2;
static volatile uint32_t g_run_start_ms = 0u;
static uint32_t g_last_menu_refresh_ms = 0u;

static const char *App_ModeLabel(AppModeId mode)
{
    switch (mode)
    {
        case APP_MODE_Q2:
            return "Q2";
        case APP_MODE_Q3:
            return "Q3";
        case APP_MODE_Q4:
            return "Q4";
        case APP_MODE_Q6:
            return "Q6";
        default:
            return "??";
    }
}

static void App_RenderMenu(void)
{
    OLED_ClearBuffer();
    OLED_DrawPrintf(0, 0, 16, ">%s  S%u",
                    App_ModeLabel(g_selected_mode),
                    (unsigned)g_run_seconds);
    OLED_DrawPrintf(0, 16, 12, "Q2 Q3 Q4 Q6");
    OLED_DrawPrintf(0, 32, 16, "K2 START");
    OLED_DrawPrintf(0, 48, 12, "K1 SELECT  K2 RUN");
    OLED_Refresh();
}

static void App_StopActive(void)
{
    g_app_state = APP_STATE_MENU;
    switch (g_active_mode)
    {
        case APP_MODE_Q2:
            ModeQ2_Stop();
            break;
        case APP_MODE_Q3:
            ModeQ3_Stop();
            break;
        case APP_MODE_Q4:
            ModeQ4_Stop();
            break;
        case APP_MODE_Q6:
            ModeQ5_Stop();
            break;
        default:
            break;
    }
    Motor_StopSafe();
    g_run_seconds = 0u;
    g_run_start_ms = system_ms;
}

static void App_StartSelected(void)
{
    UART_ResetRxMailbox();
    g_active_mode = g_selected_mode;
    switch (g_active_mode)
    {
        case APP_MODE_Q2:
            ModeQ2_Init();
            ModeQ2_Start();
            break;
        case APP_MODE_Q3:
            ModeQ3_Init();
            ModeQ3_Start();
            break;
        case APP_MODE_Q4:
            ModeQ4_Init();
            ModeQ4_Start();
            break;
        case APP_MODE_Q6:
            ModeQ5_Init();
            ModeQ5_Start();
            break;
        default:
            break;
    }
    g_run_start_ms = system_ms;
    g_run_seconds = 0u;
    g_app_state = APP_STATE_RUNNING;
}

static void App_RunLoop(void)
{
    switch (g_active_mode)
    {
        case APP_MODE_Q2:
            ModeQ2_Loop();
            break;
        case APP_MODE_Q3:
            ModeQ3_Loop();
            break;
        case APP_MODE_Q4:
            ModeQ4_Loop();
            break;
        case APP_MODE_Q6:
            ModeQ5_Loop();
            break;
        default:
            break;
    }
}

static void App_HandleKeys(void)
{
    uint8_t key = Key_GetNum();

    if (key == 1u)
    {
        if (g_app_state == APP_STATE_RUNNING)
        {
            App_StopActive();
        }
        else
        {
            g_selected_mode = (AppModeId)(
                ((int)g_selected_mode + 1) % (int)APP_MODE_COUNT);
        }
        App_RenderMenu();
    }
    else if (key == 2u)
    {
        if (g_app_state == APP_STATE_MENU)
        {
            App_StartSelected();
        }
    }
}

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    OLED_ColorTurn(0u);
    OLED_DisplayTurn(0u);
    OLED_Clear();
    Key_Init();

    Motor_StopSafe();
    Encoder_Init();
    Servo_SetAngle1(85.0f);

    NVIC_EnableIRQ(Serial_INST_INT_IRQN);
    DL_Timer_startCounter(MOTOR_INST);
    DL_Timer_startCounter(TIMER_PID_INST);
    NVIC_EnableIRQ(TIMER_PID_INST_INT_IRQN);
    DL_Timer_startCounter(SERVO_INST);

    g_app_state = APP_STATE_MENU;
    g_selected_mode = APP_MODE_Q2;
    g_active_mode = APP_MODE_Q2;
    g_run_start_ms = 0u;
    g_run_seconds = 0u;
    system_ms = 0u;

    App_RenderMenu();

    while (1)
    {
        App_HandleKeys();

        if (g_app_state == APP_STATE_RUNNING)
        {
            App_RunLoop();
        }
        else if ((system_ms - g_last_menu_refresh_ms) >= 200u)
        {
            g_last_menu_refresh_ms = system_ms;
            App_RenderMenu();
        }
    }
}

void TIMER_PID_INST_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(TIMER_PID_INST) ==
        DL_TIMER_IIDX_LOAD)
    {
        system_ms++;
        Key_Tick();

        if (g_app_state == APP_STATE_RUNNING)
        {
            g_run_seconds =
                (uint32_t)((system_ms - g_run_start_ms) / 1000u);
            switch (g_active_mode)
            {
                case APP_MODE_Q2:
                    ModeQ2_Tick1ms();
                    break;
                case APP_MODE_Q3:
                    ModeQ3_Tick1ms();
                    break;
                case APP_MODE_Q4:
                    ModeQ4_Tick1ms();
                    break;
                case APP_MODE_Q6:
                    ModeQ5_Tick1ms();
                    break;
                default:
                    break;
            }
        }
        else
        {
            g_run_seconds = 0u;
        }
    }
}
