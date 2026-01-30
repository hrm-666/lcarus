#include "hover_thrust_ekf.h"

//使用示例
//int main() {
//    HoverThrustEKF ekf;
//    const float g = 9.81f;

//    // 初始化：初始油门0.5，Q=0.01，R=0.1
//    hover_thrust_ekf_init(&ekf, 0.5f, 0.01f, 0.1f, g);

//    // 模拟输入（实际应从IMU和电机控制器获取）
//    float throttle_cmd = 0.6f;    // 当前油门指令
//    float imu_accel_z = 2.5f;     // IMU测量的Z轴加速度（m/s2）

//    // 更新EKF
//    hover_thrust_ekf_update(&ekf, throttle_cmd, imu_accel_z);

//    printf("Estimated Hover Throttle: %.3f\n", ekf.hover_throttle);
//    return 0;
//}

void hover_thrust_ekf_init(HoverThrustEKF *ekf, float init_throttle, float Q, float R, float g) {
    ekf->hover_throttle = init_throttle;  // 初始悬停油门猜测（如0.5）
    ekf->P = 1.0f;                        // 初始协方差
    ekf->Q = Q;                           // 过程噪声（建议0.001~0.01）
    ekf->R = R;                           // 观测噪声（建议0.1~1.0）
    ekf->g = g;                           // 重力加速度（通常9.81）
}

void hover_thrust_ekf_update(HoverThrustEKF *ekf, float throttle_in, float imu_accel_z) {
    // 1. 预测阶段（状态不变，协方差增加过程噪声）
    ekf->P += ekf->Q;

    // 2. 计算观测残差（实际加速度 - 预测加速度）
    float predicted_accel = (ekf->g * throttle_in / ekf->hover_throttle) - ekf->g;
    float residual = imu_accel_z - predicted_accel;

    // 3. 计算卡尔曼增益
    float C = -ekf->g * throttle_in / (ekf->hover_throttle * ekf->hover_throttle);  // 观测矩阵
    float K = ekf->P * C / (C * ekf->P * C + ekf->R);

    // 4. 状态更新
    ekf->hover_throttle += K * residual;
    ekf->P *= (1.0f - K * C);  // 协方差更新
}
