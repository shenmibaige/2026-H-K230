#ifndef __MPU6050_H
#define __MPU6050_H
void MPU6050_init(void);
void MPU6050_GetAngle(float* pitch, float* roll, float* yaw);
#endif