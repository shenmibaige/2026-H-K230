/*
 * H Q3 v3.3 - v3.2 early braking plus B-point quiet hysteresis hold.
 *
 * K230 input remains exactly "@dx\r\n" on UART0 PA31 RX at 115200 baud.
 * Servo1 remains TIMG7 CCP1 on PA27.  PA28 UART TX is diagnostics only.
 */

#include "ti_msp_dl_config.h"

#include <stdint.h>
#include <string.h>

#include "ball_config.h"
#include "ball_controller.h"
#include "ball_ui.h"
#include "key.h"
#include "oled.h"
#include "servo.h"
#include "uart.h"
#include "mode_q3.h"
#include "app_common.h"

#define UART_PACKET_COPY_SIZE 100u
#define LOG_DUMP_PERIOD_MS      20u

static volatile uint32_t g_vision_reference_ms = 0u;
static volatile uint32_t g_task_start_ms = 0u;
static volatile uint8_t g_vision_watch_enabled = 0u;
static volatile uint8_t g_task_watch_enabled = 0u;
static volatile uint8_t g_vision_timeout_event = 0u;
static volatile uint8_t g_task_timeout_event = 0u;

static BallController g_ball;
static BallUiThrottle g_ui;
static uint16_t g_last_servo10 = 0xFFFFu;
static uint32_t g_uart_rx_count = 0u;
static uint32_t g_oled_next_page_ms = 0u;
static uint8_t g_oled_b_page = 0u;

static uint8_t g_dump_active = 0u;
static uint8_t g_dump_header_sent = 0u;
static uint16_t g_dump_index = 0u;
static uint16_t g_dump_count = 0u;
static uint32_t g_dump_next_ms = 0u;
static uint8_t g_task_started_once = 0u;

static uint32_t System_Now(void)
{
    return system_ms;
}

static uint32_t ClampDisplayTime(uint32_t value_ms)
{
    return (value_ms > 9999u) ? 9999u : value_ms;
}

static int32_t Round100(float value)
{
    float scaled = value * 100.0f;
    return (int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static uint16_t ClampAge16(uint32_t age_ms)
{
    return (age_ms > 65535u) ? 65535u : (uint16_t)age_ms;
}

static float Servo_HardClamp(float angle_deg)
{
    if (angle_deg < BALL_SERVO_HARD_MIN_DEG) {
        return BALL_SERVO_HARD_MIN_DEG;
    }
    if (angle_deg > BALL_SERVO_HARD_MAX_DEG) {
        return BALL_SERVO_HARD_MAX_DEG;
    }
    return angle_deg;
}

static void Servo_ApplySafe(float requested_angle_deg, uint8_t force)
{
    float angle_deg = Servo_HardClamp(requested_angle_deg);
    uint16_t angle10 = (uint16_t)(angle_deg * 10.0f + 0.5f);

    if ((force != 0u) || (angle10 != g_last_servo10)) {
        Servo_SetAngle1(angle_deg);
        g_last_servo10 = angle10;
    }
}

static void DisableUnusedQ3Interrupts(void)
{
    /* SysConfig is preserved; unused Q2 encoder IRQ sources are disabled here. */
    NVIC_DisableIRQ(ENCODER_GPIOA_INT_IRQN);
    NVIC_DisableIRQ(ENCODER_GPIOB_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODER_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODER_GPIOB_INT_IRQN);
    DL_GPIO_disableInterrupt(ENCODER_A1_PORT, ENCODER_A1_PIN | ENCODER_B1_PIN);
    DL_GPIO_disableInterrupt(ENCODER_A2_PORT, ENCODER_A2_PIN | ENCODER_B2_PIN);
    NVIC_DisableIRQ(MOTOR_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(MOTOR_INST_INT_IRQN);

    /* The Q3 clock is TIMER_PID.  MPU/DMP SysTick is not used in this project. */
    DL_SYSTICK_disableInterrupt();
    DL_SYSTICK_disable();
}

static void SyncTimerMirrors(const BallController *controller)
{
    uint32_t vision_reference_ms = controller->has_valid_measurement ?
        controller->last_valid_ms : controller->wait_start_ms;

    g_vision_watch_enabled = 0u;
    g_task_watch_enabled = 0u;
    g_vision_reference_ms = vision_reference_ms;
    g_task_start_ms = controller->task_start_ms;
    if (BallController_VisionWatchActive(controller) != 0u) {
        g_vision_watch_enabled = 1u;
    } else {
        g_vision_timeout_event = 0u;
    }
    if (BallController_TaskTimerActive(controller) != 0u) {
        g_task_watch_enabled = 1u;
    } else {
        g_task_timeout_event = 0u;
    }
}

static uint8_t ParseSignedDx(const char *text, int16_t *dx_px)
{
    int32_t value = 0;
    int32_t sign = 1;
    uint8_t digits = 0u;

    if (*text == '-') {
        sign = -1;
        ++text;
    } else if (*text == '+') {
        ++text;
    }

    while ((*text >= '0') && (*text <= '9')) {
        value = value * 10 + (int32_t)(*text - '0');
        if (value > 32768) {
            return 0u;
        }
        digits = 1u;
        ++text;
    }
    if ((digits == 0u) || (*text != '\0')) {
        return 0u;
    }
    value *= sign;
    if ((value < -32768) || (value > 32767)) {
        return 0u;
    }
    *dx_px = (int16_t)value;
    return 1u;
}

static uint8_t UART_TakeDxPacket(int16_t *dx_px,
                                 uint8_t *valid,
                                 uint32_t *arrival_ms,
                                 uint32_t *max_interarrival_gap_ms)
{
    char packet[UART_PACKET_COPY_SIZE];

    if (UART_TakeLatestPacket(packet,
                              UART_PACKET_COPY_SIZE,
                              arrival_ms,
                              max_interarrival_gap_ms) == 0u) {
        return 0u;
    }
    *valid = ParseSignedDx(packet, dx_px);
    return 1u;
}

static void OLED_RenderStatus(const BallController *controller, uint32_t now_ms)
{
    uint32_t age_ms = BallController_DataAgeMs(controller, now_ms);
    uint32_t total_ms = BallController_TaskElapsedMs(controller, now_ms);
    uint32_t display_age_ms = (age_ms > 999u) ? 999u : age_ms;
    uint32_t servo10 = (uint32_t)(controller->servo_command_deg * 10.0f + 0.5f);
    uint32_t quiet_age_ms = 0u;
    uint8_t b_motion_phase = ((controller->state == BALL_STATE_DRIVE_B) ||
        (controller->state == BALL_STATE_B_APPROACH) ||
        (controller->state == BALL_STATE_BRAKE_B)) ? 1u : 0u;
    uint8_t b_terminal_phase =
        ((controller->state == BALL_STATE_B_SETTLE) ||
         (controller->state == BALL_STATE_DONE_HOLD) ||
         ((controller->state == BALL_STATE_TIMEOUT) &&
          ((controller->diagnostics.timeout_phase == BALL_STATE_B_SETTLE) ||
           (controller->timeout_settled != 0u)))) ? 1u : 0u;

    if ((controller->diagnostics.t_quiet_enter_ms != 0u) &&
        (total_ms >= controller->diagnostics.t_quiet_enter_ms)) {
        quiet_age_ms = total_ms - controller->diagnostics.t_quiet_enter_ms;
    }

    OLED_ClearBuffer();
    if (b_motion_phase != 0u) {
        const char *reason = (controller->state == BALL_STATE_B_APPROACH) ?
            BallController_ApproachReasonShort((BallApproachReason)
                controller->diagnostics.b_approach_reason) :
            BallController_BrakeReasonShort((BallBrakeReason)
                controller->diagnostics.b_brake_reason);
        OLED_DrawPrintf(0, 0, 12, "%s/%s R:%s",
            BallController_StateShort(controller->state), reason,
            BallController_ResultShort(controller));
        if (g_oled_b_page == 0u) {
            OLED_DrawPrintf(0, 12, 12, "X%5d S%5d",
                Round100(controller->position_cm),
                Round100(controller->velocity_cm_s));
            OLED_DrawPrintf(0, 24, 12, "G%5d U%4u",
                Round100(controller->velocity_guard_cm_s), servo10);
            OLED_DrawPrintf(0, 36, 12, "AGE%3u S%4u",
                display_age_ms, (unsigned)g_run_seconds);
            OLED_DrawPrintf(0, 48, 12, "A%4u P%4u B%4u",
                ClampDisplayTime(controller->diagnostics.t_a_ms),
                ClampDisplayTime(controller->diagnostics.t_b_approach_ms),
                ClampDisplayTime(controller->diagnostics.t_b_brake_ms));
        } else {
            OLED_DrawPrintf(0, 12, 12, "S%5d G%5d",
                Round100(controller->velocity_cm_s),
                Round100(controller->velocity_guard_cm_s));
            OLED_DrawPrintf(0, 24, 12, "D%5d Q%5d",
                Round100(controller->d_remaining_cm),
                Round100(controller->d_stop_cm));
            OLED_DrawPrintf(0, 36, 12, "L%5d S%4u",
                Round100(controller->v_limit_cm_s), (unsigned)g_run_seconds);
            OLED_DrawPrintf(0, 48, 12, "E%4u F%4u Z%2u",
                ClampDisplayTime(controller->diagnostics.t_b_band_ms),
                ClampDisplayTime(controller->diagnostics.t_done_ms),
                (unsigned)controller->diagnostics.b_cross_count);
        }
        g_oled_b_page ^= 1u;
    } else if (b_terminal_phase != 0u) {
        OLED_DrawPrintf(0, 0, 12, "%s/%s R:%s",
            (controller->state == BALL_STATE_TIMEOUT) ? "TO" : "B",
            BallController_SettleModeShort(controller->settle_mode),
            BallController_ResultShort(controller));
        OLED_DrawPrintf(0, 12, 12, "X%5d V%5d",
            Round100(controller->position_cm),
            Round100(controller->velocity_cm_s));
        OLED_DrawPrintf(0, 24, 12, "QE%4d QS%4d",
            Round100(controller->quiet_error_cm),
            Round100(controller->quiet_spread_cm));
        OLED_DrawPrintf(0, 36, 12, "QA%4u S%4u",
            ClampDisplayTime(quiet_age_ms), (unsigned)g_run_seconds);
        OLED_DrawPrintf(0, 48, 12, "EV%3u X%2u P%2u",
            (unsigned)controller->diagnostics.servo_event_count,
            (unsigned)controller->diagnostics.quiet_exit_count,
            (unsigned)controller->diagnostics.quiet_pulse_count);
    } else if (controller->state == BALL_STATE_TIMEOUT) {
        OLED_DrawPrintf(0, 0, 12, "TO/%s R:%s",
            BallController_StateShort(controller->diagnostics.timeout_phase),
            BallController_ResultShort(controller));
    } else if ((BallController_IsArmed(controller) != 0u) &&
               ((controller->state == BALL_STATE_WAIT_CENTER) ||
                (controller->state == BALL_STATE_IDLE) ||
                (controller->state == BALL_STATE_ABORTED) ||
                (controller->state == BALL_STATE_VISION_FAULT))) {
        OLED_DrawPrintf(0, 0, 12, "ARM/%s R:%s",
            BallController_IsReady(controller) ? "RDY" : "WAIT",
            BallController_ResultShort(controller));
    } else if ((BallController_IsReady(controller) != 0u) &&
               ((controller->state == BALL_STATE_WAIT_CENTER) ||
                (controller->state == BALL_STATE_IDLE) ||
                (controller->state == BALL_STATE_ABORTED) ||
                (controller->state == BALL_STATE_VISION_FAULT))) {
        OLED_DrawPrintf(0, 0, 12, "RDY/%s R:%s",
            BallController_StateShort(controller->state),
            BallController_ResultShort(controller));
    } else {
        OLED_DrawPrintf(0, 0, 12, "%s R:%s",
            BallController_StateShort(controller->state),
            BallController_ResultShort(controller));
    }
    if ((b_motion_phase == 0u) && (b_terminal_phase == 0u)) {
        OLED_DrawPrintf(0, 12, 12, "X%5d V%5d",
            Round100(controller->position_cm),
            Round100(controller->velocity_cm_s));
        OLED_DrawPrintf(0, 24, 12, "P%5d S%4u",
            Round100(controller->predicted_position_cm), servo10);
        OLED_DrawPrintf(0, 36, 12, "AGE%3u S%4u",
            display_age_ms, (unsigned)g_run_seconds);
        OLED_DrawPrintf(0, 48, 12, "A%4u B%4u F%4u",
            ClampDisplayTime(controller->diagnostics.t_a_ms),
            ClampDisplayTime(controller->diagnostics.t_b_enter_ms),
            ClampDisplayTime(controller->diagnostics.t_b_band_ms));
    }
    OLED_RefreshBegin();
}

static uint8_t LogDumpEligible(const BallController *controller)
{
    if ((controller->state == BALL_STATE_DONE_HOLD) ||
        (controller->state == BALL_STATE_ABORTED) ||
        (controller->state == BALL_STATE_VISION_FAULT)) {
        return 1u;
    }
    if ((controller->state == BALL_STATE_TIMEOUT) &&
        (controller->timeout_settled != 0u)) {
        return 1u;
    }
    return 0u;
}

static void LogDumpStop(void)
{
    g_dump_active = 0u;
    g_dump_header_sent = 0u;
    g_dump_index = 0u;
    g_dump_count = 0u;
    BallController_SetLogFrozen(&g_ball, 0u);
}


static void LogDumpService(uint32_t now_ms)
{
    BallLogRecord record;

    if ((g_dump_active != 0u) && (LogDumpEligible(&g_ball) == 0u)) {
        /* A B-point recapture or new activity always pre-empts diagnostics TX. */
        LogDumpStop();
        return;
    }

    if ((g_dump_active == 0u) ||
        ((int32_t)(now_ms - g_dump_next_ms) < 0)) {
        return;
    }

    if (g_dump_header_sent == 0u) {
        UART_Printf("#SUMMARY,tA=%lu,tBApproach=%lu,tBBrake=%lu,tB=%lu,",
            (unsigned long)g_ball.diagnostics.t_a_ms,
            (unsigned long)g_ball.diagnostics.t_b_approach_ms,
            (unsigned long)g_ball.diagnostics.t_b_brake_ms,
            (unsigned long)g_ball.diagnostics.t_b_enter_ms);
        UART_Printf("tBand=%lu,tBSet=%lu,tDone=%lu,",
            (unsigned long)g_ball.diagnostics.t_b_band_ms,
            (unsigned long)g_ball.diagnostics.t_b_settle_ms,
            (unsigned long)g_ball.diagnostics.t_done_ms);
        UART_Printf("vBEnter100=%ld,aTurn100=%ld,bOver100=%ld,bCross=%u,",
            (long)Round100(g_ball.diagnostics.b_band_entry_velocity_cm_s),
            (long)Round100(g_ball.diagnostics.a_turn_cm),
            (long)Round100(g_ball.diagnostics.b_first_overshoot_cm),
            (unsigned)g_ball.diagnostics.b_cross_count);
        UART_Printf("aReason=%u,approachReason=%u,brakeReason=%u,bReason=%u,",
            (unsigned)g_ball.diagnostics.a_transition_reason,
            (unsigned)g_ball.diagnostics.b_approach_reason,
            (unsigned)g_ball.diagnostics.b_brake_reason,
            (unsigned)g_ball.diagnostics.b_release_reason);
        UART_Printf("tQuiet=%lu,quietExit=%u,quietPulse=%u,servoEvents=%u,",
            (unsigned long)g_ball.diagnostics.t_quiet_enter_ms,
            (unsigned)g_ball.diagnostics.quiet_exit_count,
            (unsigned)g_ball.diagnostics.quiet_pulse_count,
            (unsigned)g_ball.diagnostics.servo_event_count);
        UART_Printf("quietReason=%u,maxQuietError100=%d,maxServoEvents1s=%u\r\n",
            (unsigned)g_ball.diagnostics.quiet_exit_reason,
            (int)g_ball.diagnostics.max_quiet_error100,
            (unsigned)g_ball.diagnostics.max_servo_events_in_1s);
        g_dump_header_sent = 1u;
        g_dump_next_ms = now_ms + LOG_DUMP_PERIOD_MS;
        return;
    }

    if (g_dump_header_sent == 1u) {
        UART_SendString("time_ms,state,dx_px,x100,v100,xpred100,servo10,data_age,result_flags,detail,p1x2,p2x2,p3x2,p4x2,p5x2,p6x2,w1,w2,w3,w4,w5,w6,pidfused10,pidmask,piddom,vfast100,vguard100,xguard100,drem100,dstop100,vlimit100,approach_reason,brake_reason,vB_enter100,quiet_error100,quiet_flags,servo_event_count\r\n");
        g_dump_header_sent = 2u;
        g_dump_next_ms = now_ms + LOG_DUMP_PERIOD_MS;
        return;
    }

    if (g_dump_index >= g_dump_count) {
        UART_SendString("#END\r\n");
        LogDumpStop();
        return;
    }

    if (BallController_GetLog(&g_ball, g_dump_index, &record) != 0u) {
        UART_Printf("%lu,%u,%d,%d,%d,%d,%u,%u,%u,%u,",
            (unsigned long)record.time_ms,
            (unsigned)record.state,
            (int)record.dx_px,
            (int)record.x100,
            (int)record.v100,
            (int)record.xpred100,
            (unsigned)record.servo10,
            (unsigned)record.data_age,
            (unsigned)record.result_flags,
            (unsigned)record.detail);
        UART_Printf("%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%d,%u,%u,",
            (int)record.pid_output2[0],
            (int)record.pid_output2[1],
            (int)record.pid_output2[2],
            (int)record.pid_output2[3],
            (int)record.pid_output2[4],
            (int)record.pid_output2[5],
            (unsigned)record.pid_weight255[0],
            (unsigned)record.pid_weight255[1],
            (unsigned)record.pid_weight255[2],
            (unsigned)record.pid_weight255[3],
            (unsigned)record.pid_weight255[4],
            (unsigned)record.pid_weight255[5],
            (int)record.pid_fused10,
            (unsigned)record.pid_update_mask,
            (unsigned)record.pid_dominant);
        UART_Printf("%d,%d,%d,%d,%d,%d,%u,%u,%d,",
            (int)record.vfast100,
            (int)record.vguard100,
            (int)record.xguard100,
            (int)record.drem100,
            (int)record.dstop100,
            (int)record.vlimit100,
            (unsigned)record.approach_reason,
            (unsigned)record.brake_reason,
            (int)record.vB_enter100);
        UART_Printf("%d,%u,%u\r\n",
            (int)record.quiet_error100,
            (unsigned)record.quiet_flags,
            (unsigned)record.servo_event_count);
    }
    g_dump_index++;
    g_dump_next_ms = now_ms + LOG_DUMP_PERIOD_MS;
}

void ModeQ3_Init(void)
{
    DisableUnusedQ3Interrupts();
    g_vision_reference_ms = 0u;
    g_task_start_ms = 0u;
    g_vision_watch_enabled = 0u;
    g_task_watch_enabled = 0u;
    g_vision_timeout_event = 0u;
    g_task_timeout_event = 0u;
    g_task_started_once = 0u;
    g_dump_active = 0u;
    g_dump_header_sent = 0u;
    g_dump_index = 0u;
    g_dump_count = 0u;
    g_dump_next_ms = 0u;

    BallController_Init(&g_ball, System_Now());
    BallUiThrottle_Init(&g_ui, System_Now());
    Servo_ApplySafe(g_ball.servo_command_deg, 1u);
    UART_ResetRxMailbox();

    NVIC_EnableIRQ(Serial_INST_INT_IRQN);
    DL_Timer_startCounter(TIMER_PID_INST);
    NVIC_EnableIRQ(TIMER_PID_INST_INT_IRQN);
    DL_Timer_startCounter(SERVO_INST);
    SyncTimerMirrors(&g_ball);
}

void ModeQ3_Start(void)
{
    uint32_t now_ms = System_Now();

    LogDumpStop();
    BallController_RequestStart(&g_ball, now_ms);
    Servo_ApplySafe(g_ball.servo_command_deg, 0u);
    BallUiThrottle_Request(&g_ui);
    g_task_started_once = 1u;
}

void ModeQ3_Stop(void)
{
    uint32_t now_ms = System_Now();

    UART_ResetRxMailbox();
    BallController_RequestAbort(&g_ball, now_ms);
    LogDumpStop();
    Servo_ApplySafe(g_ball.servo_command_deg, 1u);
    BallUiThrottle_Request(&g_ui);
    g_vision_watch_enabled = 0u;
    g_task_watch_enabled = 0u;
    g_vision_timeout_event = 0u;
    g_task_timeout_event = 0u;
}

uint8_t ModeQ3_IsDone(void)
{
    if (g_task_started_once == 0u)
    {
        return 0u;
    }
    switch (g_ball.state)
    {
    case BALL_STATE_DONE_HOLD:
    case BALL_STATE_TIMEOUT:
    case BALL_STATE_VISION_FAULT:
    case BALL_STATE_ABORTED:
        return 1u;
    default:
        return 0u;
    }
}

void ModeQ3_Loop(void)
{
    uint32_t now_ms = System_Now();
    int16_t dx_px = 0;
    uint8_t packet_valid = 0u;
    uint8_t packet_received;
    uint32_t packet_arrival_ms = now_ms;
    uint32_t packet_max_gap_ms = 0u;
    BallState state_before = g_ball.state;


        packet_received = UART_TakeDxPacket(&dx_px,
                                            &packet_valid,
                                            &packet_arrival_ms,
                                            &packet_max_gap_ms);
        if (packet_received != 0u) {
            uint32_t processing_age_ms;
            uint32_t silence_before_packet_ms;

            now_ms = System_Now();
            processing_age_ms = (uint32_t)(now_ms - packet_arrival_ms);
            if (g_ball.has_valid_measurement == 0u) {
                silence_before_packet_ms =
                    (uint32_t)(packet_arrival_ms - g_ball.wait_start_ms);
            } else {
                silence_before_packet_ms = packet_max_gap_ms;
            }
            g_uart_rx_count++;
            if ((packet_valid == 0u) || (dx_px == BALL_LOST_DX_PX)) {
                LogDumpStop();
                BallController_OnVisionFault(&g_ball, now_ms,
                    ClampAge16(BallController_DataAgeMs(&g_ball, now_ms)));
                Servo_ApplySafe(g_ball.servo_command_deg, 1u);
            } else if ((BallController_VisionWatchActive(&g_ball) != 0u) &&
                       ((processing_age_ms >= BALL_VISION_TIMEOUT_MS) ||
                        (silence_before_packet_ms >= BALL_VISION_TIMEOUT_MS))) {
                /* A stale packet or a real inter-arrival gap cannot erase an outage. */
                LogDumpStop();
                BallController_OnVisionFault(&g_ball, now_ms,
                    ClampAge16((processing_age_ms > silence_before_packet_ms) ?
                               processing_age_ms : silence_before_packet_ms));
                Servo_ApplySafe(g_ball.servo_command_deg, 1u);
            } else {
                g_vision_timeout_event = 0u;
                g_vision_reference_ms = packet_arrival_ms;
                BallController_OnMeasurement(&g_ball, dx_px, packet_arrival_ms);
                Servo_ApplySafe(g_ball.servo_command_deg, 0u);
            }
            BallUiThrottle_Request(&g_ui);
        }

        if ((g_vision_timeout_event != 0u) &&
            (BallController_VisionWatchActive(&g_ball) != 0u) &&
            (BallController_DataAgeMs(&g_ball, now_ms) >= BALL_VISION_TIMEOUT_MS)) {
            uint32_t age_ms = BallController_DataAgeMs(&g_ball, now_ms);
            g_vision_timeout_event = 0u;
            LogDumpStop();
            BallController_OnVisionFault(&g_ball, now_ms, ClampAge16(age_ms));
            Servo_ApplySafe(g_ball.servo_command_deg, 1u);
            BallUiThrottle_Request(&g_ui);
        }

        if (g_task_timeout_event != 0u) {
            g_task_timeout_event = 0u;
            BallController_RequestTimeout(&g_ball, now_ms);
            BallUiThrottle_Request(&g_ui);
        }

        if (state_before != g_ball.state) {
            BallUiThrottle_Request(&g_ui);
        }

        SyncTimerMirrors(&g_ball);

        /* Display is serviced only after the newest UART/control/servo work. */
        if ((OLED_RefreshBusy() == 0u) &&
            (BallUiThrottle_TakeRefresh(&g_ui, now_ms,
                                       BallController_IsActive(&g_ball)) != 0u)) {
            OLED_RenderStatus(&g_ball, now_ms);
            g_oled_next_page_ms = now_ms;
        }
        if ((OLED_RefreshBusy() != 0u) &&
            ((int32_t)(System_Now() - g_oled_next_page_ms) >= 0)) {
            (void)OLED_RefreshStep();
            g_oled_next_page_ms = System_Now() +
                                  BALL_OLED_PAGE_STEP_PERIOD_MS;
        }

        /* One CSV record per service slot; RX/control never depends on PA28. */
        LogDumpService(now_ms);
}

/*
 * Called from the shared 1 ms ISR. Sets timeout event flags only.
 */
void ModeQ3_Tick1ms(void)
{
        uint32_t now_ms = system_ms;

        if ((g_vision_watch_enabled != 0u) &&
            ((uint32_t)(now_ms - g_vision_reference_ms) >=
             BALL_VISION_TIMEOUT_MS)) {
            g_vision_timeout_event = 1u;
        }
        if ((g_task_watch_enabled != 0u) &&
            ((uint32_t)(now_ms - g_task_start_ms) >=
             BALL_OFFICIAL_TIMEOUT_MS)) {
            g_task_timeout_event = 1u;
        }
}
