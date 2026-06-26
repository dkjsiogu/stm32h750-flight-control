#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "flight_control/eval/closed_loop_evaluator.hpp"
#include "flight_control/model/static_mlp_policy.hpp"

namespace {

/** 姿态策略搜索参数数量。 */
constexpr std::size_t kParameterCount = 70;
/** 搜索参数导出文件路径。 */
constexpr const char* kParameterOutputPath = "tools/static_policy_params.txt";
/** 每轴线性历史参数数量。 */
constexpr std::size_t kLinearParamsPerAxis = 9;
/** 每轴非线性分段参数数量。 */
constexpr std::size_t kNonlinearParamsPerAxis = 14;

/** 搜索参数数组。 */
using ParameterVector = std::array<float, kParameterCount>;

/**
 * 单次搜索结果。
 *
 * 保存参数、每个场景结果、平均分、最差分和稳定场景数量。
 */
struct SearchResult {
    /** 被评估的策略参数。 */
    ParameterVector params{};
    /** 所有场景平均分。 */
    float average_score{0.0f};
    /** 所有场景中的最低分。 */
    float worst_score{0.0f};
    /** 稳定通过的场景数量。 */
    int stable_count{0};
    /** 每个场景的完整评估结果。 */
    std::vector<flight_control::EvaluationResult> results{};
};

/**
 * 可导出的 128-128 历史姿态策略。
 *
 * 该策略与 export_linear_policy.py 的权重展开保持一致：先使用历史误差、
 * 角速度和上一动作构造线性项，再叠加可由 ReLU 表达的分段非线性门控项。
 */
class ExportableHistoryPolicy final : public flight_control::IAttitudePolicy {
public:
    /**
     * 构造策略。
     *
     * @param params 搜索得到的策略参数。
     */
    explicit ExportableHistoryPolicy(ParameterVector params)
        : params_(params) {}

    /**
     * 预测三轴归一化力矩动作。
     *
     * @param input 历史窗口展开后的模型输入。
     * @return 三轴归一化动作。
     */
    std::array<float, 3> predict(const std::array<float, flight_control::kModelInputDim>& input) override {
        auto frame = [&input](std::size_t frame_index, std::size_t value_index) {
            return input[frame_index * flight_control::kFrameDim + value_index];
        };

        std::array<float, 3> action{};
        for (std::size_t axis = 0; axis < action.size(); ++axis) {
            const std::size_t offset = axis * kLinearParamsPerAxis;
            const float error_now = frame(flight_control::kHistoryFrames - 1U, axis);
            const float error_prev = frame(flight_control::kHistoryFrames - 2U, axis);
            const float error_old = frame(flight_control::kHistoryFrames - 3U, axis);
            const float omega_now = frame(flight_control::kHistoryFrames - 1U, axis + 3U);
            const float omega_prev = frame(flight_control::kHistoryFrames - 2U, axis + 3U);
            const float previous_action = frame(flight_control::kHistoryFrames - 1U, axis + 6U);
            const float error_velocity = error_now - error_prev;
            const float error_curve = error_now - 2.0f * error_prev + error_old;
            const float omega_velocity = omega_now - omega_prev;
            const float linear =
                -params_[offset + 0U] * error_now -
                params_[offset + 1U] * omega_now -
                params_[offset + 2U] * error_velocity -
                params_[offset + 3U] * error_curve -
                params_[offset + 4U] * omega_velocity -
                params_[offset + 5U] * previous_action +
                params_[offset + 6U] * error_prev +
                params_[offset + 7U];

            const std::size_t gate_offset = 28U + axis * kNonlinearParamsPerAxis;
            const float nonlinear =
                params_[gate_offset + 3U] * dead_abs(error_now, params_[gate_offset + 0U]) +
                params_[gate_offset + 4U] * dead_abs(omega_now, params_[gate_offset + 1U]) +
                params_[gate_offset + 5U] * dead_abs(previous_action, params_[gate_offset + 2U]) +
                params_[gate_offset + 7U] * dead_abs(error_now, params_[gate_offset + 6U]) +
                params_[gate_offset + 9U] * dead_abs(omega_now, params_[gate_offset + 8U]) +
                params_[gate_offset + 11U] * dead_abs(previous_action, params_[gate_offset + 10U]) +
                params_[gate_offset + 12U];

            action[axis] = std::tanh(params_[27U] * linear + params_[offset + 8U] + params_[69U] * nonlinear);
        }
        return action;
    }

private:
    /**
     * 计算带死区绝对值特征。
     *
     * @param value 原始输入。
     * @param threshold 死区阈值。
     * @return max(value-threshold, 0) + max(-value-threshold, 0)。
     */
    static float dead_abs(float value, float threshold) {
        const float limit = std::max(0.0f, threshold);
        return std::max(0.0f, value - limit) + std::max(0.0f, -value - limit);
    }

    /** 策略参数。 */
    ParameterVector params_{};
};

/**
 * 计算平均分。
 *
 * @param results 场景结果列表。
 * @return 平均场景分。
 */
float average_score(const std::vector<flight_control::EvaluationResult>& results) {
    float total = 0.0f;
    for (const auto& result : results) {
        total += result.metrics.score;
    }
    return results.empty() ? 0.0f : total / static_cast<float>(results.size());
}

/**
 * 闭环评估一组策略参数。
 *
 * @param params 策略参数。
 * @return 搜索结果。
 */
SearchResult evaluate(const ParameterVector& params) {
    auto policy = std::make_shared<ExportableHistoryPolicy>(params);
    flight_control::ModelAdapterConfig model_config{};
    model_config.torque_limit_nm = 0.15f;
    model_config.target_reset_threshold = 2.0f;
    model_config.normalization.omega_mean = {0.0f, 0.0f, 0.0f};
    model_config.normalization.omega_std = {1.0f, 1.0f, 1.0f};

    flight_control::ClosedLoopEvaluator evaluator(policy);
    SearchResult result{};
    result.params = params;
    result.worst_score = std::numeric_limits<float>::max();
    for (const auto& scenario : flight_control::default_evaluation_scenarios(model_config)) {
        result.results.push_back(evaluator.run(scenario));
    }
    result.average_score = average_score(result.results);
    for (const auto& scenario_result : result.results) {
        result.worst_score = std::min(result.worst_score, scenario_result.metrics.score);
        result.stable_count += scenario_result.metrics.stable ? 1 : 0;
    }
    return result;
}

/**
 * 搜索目标函数。
 *
 * @param result 单次评估结果。
 * @return 目标分，越高越好。
 */
float objective(const SearchResult& result) {
    return result.average_score +
           0.42f * result.worst_score +
           8.0f * static_cast<float>(result.stable_count);
}

/**
 * 限制搜索参数范围。
 *
 * @param params 原始候选参数。
 * @return 限幅后的候选参数。
 */
ParameterVector clamp_params(ParameterVector params) {
    for (std::size_t axis = 0; axis < 3U; ++axis) {
        const std::size_t offset = axis * kLinearParamsPerAxis;
        params[offset + 0U] = std::clamp(params[offset + 0U], 0.1f, 18.0f);
        params[offset + 1U] = std::clamp(params[offset + 1U], 0.01f, 5.5f);
        params[offset + 2U] = std::clamp(params[offset + 2U], -12.0f, 12.0f);
        params[offset + 3U] = std::clamp(params[offset + 3U], -9.0f, 9.0f);
        params[offset + 4U] = std::clamp(params[offset + 4U], -6.0f, 6.0f);
        params[offset + 5U] = std::clamp(params[offset + 5U], -1.4f, 1.4f);
        params[offset + 6U] = std::clamp(params[offset + 6U], -6.0f, 6.0f);
        params[offset + 7U] = std::clamp(params[offset + 7U], -0.5f, 0.5f);
        params[offset + 8U] = std::clamp(params[offset + 8U], -0.5f, 0.5f);

        const std::size_t gate_offset = 28U + axis * kNonlinearParamsPerAxis;
        params[gate_offset + 0U] = std::clamp(params[gate_offset + 0U], 0.0f, 0.18f);
        params[gate_offset + 1U] = std::clamp(params[gate_offset + 1U], 0.0f, 1.8f);
        params[gate_offset + 2U] = std::clamp(params[gate_offset + 2U], 0.0f, 0.65f);
        params[gate_offset + 3U] = std::clamp(params[gate_offset + 3U], -8.0f, 8.0f);
        params[gate_offset + 4U] = std::clamp(params[gate_offset + 4U], -2.5f, 2.5f);
        params[gate_offset + 5U] = std::clamp(params[gate_offset + 5U], -2.0f, 2.0f);
        params[gate_offset + 6U] = std::clamp(params[gate_offset + 6U], 0.0f, 0.32f);
        params[gate_offset + 7U] = std::clamp(params[gate_offset + 7U], -8.0f, 8.0f);
        params[gate_offset + 8U] = std::clamp(params[gate_offset + 8U], 0.0f, 2.8f);
        params[gate_offset + 9U] = std::clamp(params[gate_offset + 9U], -2.5f, 2.5f);
        params[gate_offset + 10U] = std::clamp(params[gate_offset + 10U], 0.0f, 0.95f);
        params[gate_offset + 11U] = std::clamp(params[gate_offset + 11U], -2.0f, 2.0f);
        params[gate_offset + 12U] = std::clamp(params[gate_offset + 12U], -0.20f, 0.20f);
        params[gate_offset + 13U] = std::clamp(params[gate_offset + 13U], -1.0f, 1.0f);
    }
    params[27U] = std::clamp(params[27U], 0.25f, 2.2f);
    params[69U] = std::clamp(params[69U], -1.3f, 1.3f);
    return params;
}

/**
 * 写出最优参数，供 Python 权重导出器读取。
 *
 * @param params 最优参数。
 */
void write_params(const ParameterVector& params) {
    std::ofstream output(kParameterOutputPath);
    output << std::setprecision(9);
    for (std::size_t index = 0; index < params.size(); ++index) {
        output << params[index] << (index + 1U == params.size() ? '\n' : ' ');
    }
}

/**
 * 打印搜索结果。
 *
 * @param label 输出标签。
 * @param result 搜索结果。
 */
void print_result(const std::string& label, const SearchResult& result) {
    std::cout << label
              << " stable=" << result.stable_count << "/" << result.results.size()
              << " avg=" << result.average_score
              << " worst=" << result.worst_score
              << " params=";
    for (const float value : result.params) {
        std::cout << value << ' ';
    }
    std::cout << '\n';
}

/**
 * 创建上一轮线性搜索得到的起始参数。
 *
 * @return 初始参数。
 */
ParameterVector initial_params() {
    ParameterVector params{};
    std::ifstream input(kParameterOutputPath);
    if (input.good()) {
        bool loaded = true;
        for (float& value : params) {
            if (!(input >> value)) {
                loaded = false;
                break;
            }
        }
        if (loaded) {
            return clamp_params(params);
        }
    }

    const std::array<float, 28> linear{
        5.86136f, 0.806418f, -4.40802f, -2.08314f, 5.41356f, -0.663206f, -5.68171f, -0.0286817f, 0.0489095f,
        12.9824f, 1.22895f, -3.17230f, -2.02061f, 5.98654f, -0.582643f, -4.12559f, -0.0427179f, 0.0417726f,
        12.3348f, 0.818412f, -3.86000f, -3.51296f, 4.56577f, -0.406924f, -3.05539f, 0.00586839f, -0.0431760f,
        1.21296f,
    };
    std::copy(linear.begin(), linear.end(), params.begin());
    for (std::size_t axis = 0; axis < 3U; ++axis) {
        const std::size_t gate_offset = 28U + axis * kNonlinearParamsPerAxis;
        params[gate_offset + 0U] = 0.035f;
        params[gate_offset + 1U] = 0.18f;
        params[gate_offset + 2U] = 0.12f;
        params[gate_offset + 3U] = 0.0f;
        params[gate_offset + 4U] = 0.0f;
        params[gate_offset + 5U] = 0.0f;
        params[gate_offset + 6U] = 0.090f;
        params[gate_offset + 7U] = 0.0f;
        params[gate_offset + 8U] = 0.65f;
        params[gate_offset + 9U] = 0.0f;
        params[gate_offset + 10U] = 0.36f;
        params[gate_offset + 11U] = 0.0f;
        params[gate_offset + 12U] = 0.0f;
        params[gate_offset + 13U] = 0.0f;
    }
    params[69U] = 0.35f;
    return clamp_params(params);
}

/**
 * 创建每个参数的搜索标准差。
 *
 * @return 标准差数组。
 */
ParameterVector initial_sigma() {
    ParameterVector sigma{};
    for (std::size_t axis = 0; axis < 3U; ++axis) {
        const std::size_t offset = axis * kLinearParamsPerAxis;
        sigma[offset + 0U] = 0.45f;
        sigma[offset + 1U] = 0.08f;
        sigma[offset + 2U] = 0.22f;
        sigma[offset + 3U] = 0.20f;
        sigma[offset + 4U] = 0.22f;
        sigma[offset + 5U] = 0.035f;
        sigma[offset + 6U] = 0.28f;
        sigma[offset + 7U] = 0.012f;
        sigma[offset + 8U] = 0.012f;

        const std::size_t gate_offset = 28U + axis * kNonlinearParamsPerAxis;
        sigma[gate_offset + 0U] = 0.015f;
        sigma[gate_offset + 1U] = 0.070f;
        sigma[gate_offset + 2U] = 0.045f;
        sigma[gate_offset + 3U] = 0.40f;
        sigma[gate_offset + 4U] = 0.16f;
        sigma[gate_offset + 5U] = 0.14f;
        sigma[gate_offset + 6U] = 0.030f;
        sigma[gate_offset + 7U] = 0.34f;
        sigma[gate_offset + 8U] = 0.18f;
        sigma[gate_offset + 9U] = 0.14f;
        sigma[gate_offset + 10U] = 0.08f;
        sigma[gate_offset + 11U] = 0.12f;
        sigma[gate_offset + 12U] = 0.015f;
        sigma[gate_offset + 13U] = 0.0f;
    }
    sigma[27U] = 0.035f;
    sigma[69U] = 0.10f;
    return sigma;
}

}  // namespace

/**
 * 离线搜索入口。
 *
 * @return 所有场景稳定时返回 0。
 */
int main() {
    std::mt19937 rng(20260626U);
    ParameterVector best_params = initial_params();
    SearchResult best = evaluate(best_params);
    print_result("seed", best);

    ParameterVector sigma = initial_sigma();
    for (int generation = 0; generation < 220; ++generation) {
        const int population = generation < 70 ? 46 : (generation < 150 ? 34 : 24);
        for (int sample = 0; sample < population; ++sample) {
            ParameterVector candidate = best_params;
            for (std::size_t index = 0; index < candidate.size(); ++index) {
                std::normal_distribution<float> noise(0.0f, sigma[index]);
                candidate[index] += noise(rng);
            }
            candidate = clamp_params(candidate);
            SearchResult result = evaluate(candidate);
            if (objective(result) > objective(best)) {
                best = result;
                best_params = candidate;
                print_result("best", best);
                write_params(best_params);
            }
        }
        for (float& value : sigma) {
            value *= 0.986f;
        }
        if (generation % 25 == 24) {
            print_result("progress", best);
            write_params(best_params);
        }
    }

    write_params(best_params);
    std::cout << flight_control::format_metrics_table(best.results);
    print_result("final", best);
    return best.stable_count == static_cast<int>(best.results.size()) ? 0 : 1;
}
