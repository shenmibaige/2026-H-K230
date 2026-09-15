#include "ti_msp_dl_config.h"
#include "motor.h"

static int16_t Motor_ClampSpeed(int16_t speed)
{
    if (speed > 100)
    {
        return 100;
    }
    if (speed < -100)
    {
        return -100;
    }
    return speed;
}

/**
 * Motor1 = 右后轮。
 *
 * 该电机因镜像安装，正 PWM 的电气方向必须保持 gap-hold 实车版已经
 * 验证的 AIN1=0、AIN2=1；不要与 Motor2 的正向引脚组合改成相同。
 */
void Motor1_SetSpeed(int16_t Speed)
{
    Speed = Motor_ClampSpeed(Speed);

    if (Speed >= 0)
    {
        DL_GPIO_clearPins(
            DC_MOTOR_AIN1_PORT,
            DC_MOTOR_AIN1_PIN);
        DL_GPIO_setPins(
            DC_MOTOR_AIN2_PORT,
            DC_MOTOR_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(
            MOTOR_INST,
            (uint32_t)Speed,
            GPIO_MOTOR_C0_IDX);
    }
    else
    {
        DL_GPIO_setPins(
            DC_MOTOR_AIN1_PORT,
            DC_MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(
            DC_MOTOR_AIN2_PORT,
            DC_MOTOR_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(
            MOTOR_INST,
            (uint32_t)(-Speed),
            GPIO_MOTOR_C0_IDX);
    }
}

/**
 * Motor2 = 左后轮，保持 gap-hold 实车版的 BIN1=1、BIN2=0 正向定义。
 */
void Motor2_SetSpeed(int16_t Speed)
{
    Speed = Motor_ClampSpeed(Speed);

    if (Speed >= 0)
    {
        DL_GPIO_setPins(
            DC_MOTOR_BIN1_PORT,
            DC_MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(
            DC_MOTOR_BIN2_PORT,
            DC_MOTOR_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(
            MOTOR_INST,
            (uint32_t)Speed,
            GPIO_MOTOR_C1_IDX);
    }
    else
    {
        DL_GPIO_clearPins(
            DC_MOTOR_BIN1_PORT,
            DC_MOTOR_BIN1_PIN);
        DL_GPIO_setPins(
            DC_MOTOR_BIN2_PORT,
            DC_MOTOR_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(
            MOTOR_INST,
            (uint32_t)(-Speed),
            GPIO_MOTOR_C1_IDX);
    }
}

void Motor1_Brake(void)
{
    DL_Timer_setCaptureCompareValue(
        MOTOR_INST,
        0U,
        GPIO_MOTOR_C0_IDX);
    DL_GPIO_setPins(
        DC_MOTOR_AIN1_PORT,
        DC_MOTOR_AIN1_PIN);
    DL_GPIO_setPins(
        DC_MOTOR_AIN2_PORT,
        DC_MOTOR_AIN2_PIN);
}

void Motor2_Brake(void)
{
    DL_Timer_setCaptureCompareValue(
        MOTOR_INST,
        0U,
        GPIO_MOTOR_C1_IDX);
    DL_GPIO_setPins(
        DC_MOTOR_BIN1_PORT,
        DC_MOTOR_BIN1_PIN);
    DL_GPIO_setPins(
        DC_MOTOR_BIN2_PORT,
        DC_MOTOR_BIN2_PIN);
}

void Motor_StopSafe(void)
{
    Motor1_Brake();
    Motor2_Brake();
}
