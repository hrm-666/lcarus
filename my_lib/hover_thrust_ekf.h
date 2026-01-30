#ifndef _HOVER_THRUST_EKF_H_
#define _HOVER_THRUST_EKF_H_

#include "stm32f10x.h"

typedef struct {
    float hover_throttle;   // 估计的悬停油门值 (0~1)
    float P;                // 状态协方差
    float Q;                // 过程噪声协方差
    float R;                // 观测噪声协方差
    float g;                // 重力加速度 (m/s2)
} HoverThrustEKF;

// 初始化滤波器
void hover_thrust_ekf_init(HoverThrustEKF *ekf, float init_throttle, float Q, float R, float g);

// 更新EKF状态（需传入当前油门和IMU的垂直加速度）
void hover_thrust_ekf_update(HoverThrustEKF *ekf, float throttle_in, float imu_accel_z);

#endif
