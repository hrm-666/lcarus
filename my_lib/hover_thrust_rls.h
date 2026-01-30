#ifndef _HOVER_THRUST_RLS_H_
#define _HOVER_THRUST_RLS_H_

#include "stm32f10x.h"

typedef struct {
    float thr2acc;      // 油门到加速度的映射系数 (thr2acc = g / thr_hover)
    float P;            // 协方差矩阵（标量简化版）
    float lambda;       // 遗忘因子（0 < lambda <= 1）
    float g;            // 重力加速度 (m/s2)
    float min_throttle; // 油门最小有效值
    float max_throttle; // 油门最大有效值
} HoverThrustRLS;

// 初始化RLS估计器
void hover_thrust_rls_init(HoverThrustRLS *rls, float lambda, float g, float init_thr_hover);

// 更新RLS状态（需传入当前油门和IMU的垂直加速度）
void hover_thrust_rls_update(HoverThrustRLS *rls, float throttle, float imu_accel_z);

// 获取当前估计的悬停油门
float get_hover_throttle(const HoverThrustRLS *rls);

#endif
