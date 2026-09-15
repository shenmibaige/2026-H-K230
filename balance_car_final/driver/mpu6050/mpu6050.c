#include "ti_msp_dl_config.h"
#include "mpu_port.h"
/**
  * 函    数：MPU6050初始化
  * 参    数：无
  * 返 回 值：无
  */
void MPU6050_init(void)
{
    while(DMP_Init());                      
}
/**
  * 函    数：读取姿态角(滚转角 X轴,俯仰角 Y轴,偏航角 Z轴)
  * 参    数：无
  * 返 回 值：无
  */
void MPU6050_GetAngle(float* pitch, float* roll, float* yaw)
{
    while(DMP_Read_Data(pitch, roll, yaw));
    //*yaw-=10;           //防止温漂
}
extern volatile uint32_t sys_tick_ms;
void SysTick_Handler(void)
{
    sys_tick_ms++;
}