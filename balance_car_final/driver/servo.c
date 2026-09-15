#include "ti_msp_dl_config.h"
#include "servo.h"

/*单片机主频80MHz，TIMERG7预分频为80，自动重装器的值为20000*/


/**
  * 函    数：舵机设置角度
  * 参    数：Angle 要设置的舵机角度，范围：0~180
  * 返 回 值：无
  */
//舵机1角度，范围：0~180
void Servo_SetAngle1(float Angle)
{
    /* 舵机限幅：20~150度 */
    if (Angle < 20.0f) Angle = 20.0f;
    if (Angle > 150.0f) Angle = 150.0f;
    //设置占空比
    DL_Timer_setCaptureCompareValue(SERVO_INST , Angle / 180 * 2000 + 500 , GPIO_SERVO_C1_IDX);//PA27引脚
	//将角度线性变换，对应到舵机要求的占空比范围上
}
//舵机2角度，范围：0~270
void Servo_SetAngle2(float Angle)
{
	//设置占空比
    DL_Timer_setCaptureCompareValue(SERVO_INST , Angle / 270 * 2000 + 500 , GPIO_SERVO_C0_IDX);//PA26引脚
	//将角度线性变换，对应到舵机要求的占空比范围上
}
