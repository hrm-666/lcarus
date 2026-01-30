#include "hover_thrust_rls.h"

//使用示例
//int main() {
//    HoverThrustRLS rls;
//    const float g = 9.81f;

//    // 初始化：遗忘因子0.98，初始悬停油门猜测0.5
//    hover_thrust_rls_init(&rls, 0.98f, g, 0.5f);

//    // 模拟输入序列（实际应从IMU和电机控制器获取）
//    float throttle_cmds[] = {0.6f, 0.55f, 0.58f, 0.62f};
//    float imu_accels_z[] = {2.3f, 1.8f, 2.1f, 2.4f}; // 实际加速度 = raw_accel_z - g

//    for (int i = 0; i < 4; i++) {
//        hover_thrust_rls_update(&rls, throttle_cmds[i], imu_accels_z[i]);
//        printf("Step %d: Hover Throttle = %.3f\n", i, get_hover_throttle(&rls));
//    }
//    return 0;
//}

void hover_thrust_rls_init(HoverThrustRLS *rls, float lambda, float g, float init_thr_hover) {
    rls->thr2acc = g / init_thr_hover;  // 初始系数：thr2acc = g / thr_hover
    rls->P = 1.0f;                      // 初始协方差
    rls->lambda = lambda;               // 遗忘因子（通常0.95~0.99）
    rls->g = g;                         // 重力加速度
    rls->min_throttle = 0.2f;           // 默认油门有效范围
    rls->max_throttle = 0.8f;
}

void hover_thrust_rls_update(HoverThrustRLS *rls, float throttle, float imu_accel_z) {
    // 1. 检查油门输入是否有效
    if (throttle < rls->min_throttle || throttle > rls->max_throttle) {
        return; // 跳过无效数据
    }

    // 2. 计算增益系数K
    float K = rls->P * throttle / (rls->lambda + throttle * rls->P * throttle);

    // 3. 更新参数估计
    float residual = imu_accel_z - (rls->thr2acc * throttle - rls->g);
    rls->thr2acc += K * residual;

    // 4. 更新协方差
    rls->P = (1.0f - K * throttle) * rls->P / rls->lambda;

    // 5. 数值保护（避免除零或发散）
    if (rls->P > 1e6f) {
        rls->P = 1.0f; // 重置协方差
    }
    if (rls->thr2acc < 1e-3f) {
        rls->thr2acc = rls->g / 0.5f; // 重置为默认值（假设thr_hover=0.5）
    }
}

float get_hover_throttle(const HoverThrustRLS *rls) {
    return rls->g / rls->thr2acc; // thr_hover = g / thr2acc
}
