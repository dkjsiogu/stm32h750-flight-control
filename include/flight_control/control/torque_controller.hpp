#pragma once

#include "flight_control/data/flight_types.hpp"

namespace flight_control {

/**
 * 力矩控制器配置。
 *
 * 定义机架几何、推力上限和 PWM 输出约束。
 * 力矩控制器把 collective 和机体系力矩混控成四路电机 PWM。
 */
struct TorqueControllerConfig {
    /** 电机到机体中心的力臂长度，单位 m。 */
    float arm_length_m{0.23f};
    /** 偏航力矩系数，用于把 yaw 力矩换算到各电机推力差。 */
    float yaw_torque_coeff{0.018f};
    /** 最大总推力，单位 N。 */
    float max_total_thrust_n{25.506f};
    /** 单个电机允许的最大推力，单位 N。 */
    float max_motor_thrust_n{6.3765f};
    /** PWM 最小脉宽，单位 us。 */
    float pwm_min_us{1000.0f};
    /** PWM 最大脉宽，单位 us。 */
    float pwm_max_us{2000.0f};
    /** 推力曲线指数，大于 1 表示低油门段单位 PWM 产生的推力更弱。 */
    float thrust_curve_exponent{1.0f};
    /** PWM 变化斜率限制，单位 us/s。 */
    float pwm_slew_rate_us_per_sec{3500.0f};
};

/**
 * 力矩到 PWM 混控器。
 *
 * 负责把总推力和机体系力矩转换为四路电机 PWM，
 * 同时加入 PWM 斜率限制以模拟真实执行链路的响应延迟。
 */
class TorqueController {
public:
    /**
     * 构造力矩控制器。
     *
     * @param config 力矩控制器参数配置。
     */
    explicit TorqueController(TorqueControllerConfig config = {});

    /** 复位内部 PWM 记忆。 */
    void reset();
    /**
     * 使用总推力和力矩指令进行混控。
     *
     * @param collective 归一化总推力，范围通常为 0 到 1。
     * @param torque 机体系力矩指令。
     * @param dt_sec 控制周期，单位秒。
     * @return 四路电机 PWM 输出。
     */
    MotorPwmFrame mix(float collective, const TorqueCommand& torque, float dt_sec);
    /**
     * 使用总推力和机体系力矩向量进行混控。
     *
     * @param collective 归一化总推力，范围通常为 0 到 1。
     * @param body_torque_nm 机体系力矩，单位 N*m。
     * @param dt_sec 控制周期，单位秒。
     * @return 四路电机 PWM 输出。
     */
    MotorPwmFrame mix(float collective, const Vector3& body_torque_nm, float dt_sec);

private:
    /**
     * 把单个电机推力转换为 PWM，并执行斜率限制。
     *
     * @param thrust_n 目标推力，单位 N。
     * @param dt_sec 控制周期，单位秒。
     * @param motor_index 电机编号，用于访问上一帧 PWM。
     * @return 电机对应的 PWM 脉宽，单位 us。
     */
    float thrust_to_pwm(float thrust_n, float dt_sec, std::size_t motor_index);

    /** 力矩控制器配置。 */
    TorqueControllerConfig config_;
    /** 上一帧输出的 PWM 记忆。 */
    MotorPwmFrame last_pwm_{};
};

}  // namespace flight_control
