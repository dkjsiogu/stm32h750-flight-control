#pragma once

#include <memory>
#include <string>
#include <vector>

#include "flight_control/control/speed_controller.hpp"
#include "flight_control/control/torque_controller.hpp"
#include "flight_control/model/attitude_policy.hpp"
#include "flight_control/model/model_adapter.hpp"
#include "flight_control/platform/host/host_environment.hpp"

namespace flight_control {

/**
 * 分段指令。
 *
 * 表示从某个 host 评估时间开始生效的一条 GuidanceCommand。
 */
struct CommandSegment {
    /** 指令开始生效时间，单位秒。 */
    float start_sec{0.0f};
    /** 从 start_sec 起使用的指令内容。 */
    GuidanceCommand command{};
};

/**
 * 风场阶跃。
 *
 * 表示从某个 host 评估时间开始切换到新的风速。
 */
struct WindStep {
    /** 风场切换时间，单位秒。 */
    float start_sec{0.0f};
    /** 切换后的风速，单位 m/s。 */
    Vector3 wind_m_s{};
};

/**
 * 评估指标阈值。
 *
 * 用于判断一个闭环评估场景是否稳定通过。
 */
struct MetricLimits {
    /** 最大允许速度 RMS 误差，单位 m/s。 */
    float max_velocity_rms_m_s{0.55f};
    /** 最大允许高度漂移，单位 m。 */
    float max_altitude_drift_m{1.0f};
    /** 最大允许倾角，单位 rad。 */
    float max_tilt_rad{0.75f};
    /** 最大允许 PWM 饱和比例。 */
    float max_pwm_saturation_ratio{0.12f};
    /** 最大允许恢复时间，单位秒。 */
    float max_recovery_time_sec{2.0f};
};

/**
 * 闭环评估场景。
 *
 * 定义 host 真实环境时长、物理模型、控制器配置、指令序列和风场扰动。
 */
struct EvaluationScenario {
    /** 场景名称。 */
    std::string name{};
    /** host 评估总时长，单位秒。 */
    float duration_sec{8.0f};
    /** 指标统计开始时间，单位秒。 */
    float metrics_start_sec{1.0f};
    /** 恢复时间统计起点，单位秒。 */
    float recovery_start_sec{0.0f};
    /** host 真实环境参数。 */
    HostEnvironmentConfig environment_config{};
    /** 速度控制器参数。 */
    SpeedControllerConfig speed_config{};
    /** 力矩控制器参数。 */
    TorqueControllerConfig torque_config{};
    /** 模型适配器参数。 */
    ModelAdapterConfig model_config{};
    /** 稳定性阈值。 */
    MetricLimits limits{};
    /** 分段指令列表。 */
    std::vector<CommandSegment> commands{};
    /** 风场阶跃列表。 */
    std::vector<WindStep> wind_steps{};
};

/**
 * 单个场景的闭环评估指标。
 *
 * 保存闭环评估后的速度误差、高度漂移、姿态误差、PWM 饱和和最终状态。
 */
struct EvaluationMetrics {
    /** 场景名称。 */
    std::string scenario_name{};
    /** 综合评分，范围 0 到 100。 */
    float score{0.0f};
    /** 三轴速度 RMS 误差，单位 m/s。 */
    float velocity_rms_m_s{0.0f};
    /** 水平速度 RMS 误差，单位 m/s。 */
    float horizontal_velocity_rms_m_s{0.0f};
    /** 竖直速度 RMS 误差，单位 m/s。 */
    float vertical_velocity_rms_m_s{0.0f};
    /** 相对初始高度的最大漂移，单位 m。 */
    float altitude_drift_m{0.0f};
    /** 姿态误差 RMS，单位 rad。 */
    float attitude_error_rms_rad{0.0f};
    /** 最大倾角，单位 rad。 */
    float max_tilt_rad{0.0f};
    /** PWM 饱和样本比例。 */
    float pwm_saturation_ratio{0.0f};
    /** 从扰动或指令变化到稳定的恢复时间，单位秒。 */
    float recovery_time_sec{0.0f};
    /** host 评估结束位置，单位 m。 */
    Vector3 final_position_m{};
    /** host 评估结束速度，单位 m/s。 */
    Vector3 final_velocity_m_s{};
    /** 是否满足该场景的稳定性阈值。 */
    bool stable{false};
};

/**
 * 单个场景的完整评估结果。
 *
 * 同时保留场景定义和计算出的指标，便于报告输出。
 */
struct EvaluationResult {
    /** 被执行的评估场景。 */
    EvaluationScenario scenario{};
    /** 场景运行后的评估指标。 */
    EvaluationMetrics metrics{};
};

/**
 * 闭环评估运行器。
 *
 * 使用同一套 SpeedController、ModelAdapter、TorqueController 和 HostFlightEnvironment
 * 对指定场景进行确定性闭环评估。
 */
class ClosedLoopEvaluator {
public:
    /**
     * 构造评估运行器。
     *
     * @param policy 姿态策略，通常为导出的静态 MLP。
     */
    explicit ClosedLoopEvaluator(std::shared_ptr<IAttitudePolicy> policy);

    /**
     * 运行一个评估场景。
     *
     * @param scenario 场景定义。
     * @return 场景和指标组成的评估结果。
     */
    EvaluationResult run(const EvaluationScenario& scenario) const;

private:
    /** 闭环评估使用的姿态策略。 */
    std::shared_ptr<IAttitudePolicy> policy_;
};

/**
 * 获取当前调优后的速度控制器默认配置。
 *
 * @return 速度控制器配置。
 */
SpeedControllerConfig optimized_speed_controller_config();
/**
 * 获取当前调优后的力矩控制器默认配置。
 *
 * @return 力矩控制器配置。
 */
TorqueControllerConfig optimized_torque_controller_config();
/**
 * 生成默认评估场景集合。
 *
 * @param model_config 姿态模型适配器配置。
 * @return 默认闭环评估场景列表。
 */
std::vector<EvaluationScenario> default_evaluation_scenarios(const ModelAdapterConfig& model_config);
/**
 * 将评估结果格式化为 CSV 风格文本表。
 *
 * @param results 评估结果列表。
 * @return 可打印的指标表。
 */
std::string format_metrics_table(const std::vector<EvaluationResult>& results);
/**
 * 将评估结果格式化为 Markdown 报告。
 *
 * @param results 评估结果列表。
 * @return Markdown 报告文本。
 */
std::string format_markdown_report(const std::vector<EvaluationResult>& results);

}  // namespace flight_control
