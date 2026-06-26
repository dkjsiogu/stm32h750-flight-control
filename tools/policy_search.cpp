#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "flight_control/eval/closed_loop_evaluator.hpp"
#include "flight_control/model/static_mlp_policy.hpp"

namespace {

using ParameterVector = std::array<float, 28>;

struct SearchResult {
    ParameterVector params{};
    float average_score{0.0f};
    float worst_score{0.0f};
    int stable_count{0};
    std::vector<flight_control::EvaluationResult> results{};
};

class LinearHistoryPolicy final : public flight_control::IAttitudePolicy {
public:
    explicit LinearHistoryPolicy(ParameterVector params)
        : params_(params) {}

    std::array<float, 3> predict(const std::array<float, flight_control::kModelInputDim>& input) override {
        auto frame = [&input](std::size_t frame_index, std::size_t value_index) {
            return input[frame_index * flight_control::kFrameDim + value_index];
        };

        std::array<float, 3> action{};
        for (std::size_t axis = 0; axis < action.size(); ++axis) {
            const std::size_t offset = axis * 9U;
            const float error_now = frame(flight_control::kHistoryFrames - 1, axis);
            const float error_prev = frame(flight_control::kHistoryFrames - 2, axis);
            const float error_old = frame(flight_control::kHistoryFrames - 3, axis);
            const float omega_now = frame(flight_control::kHistoryFrames - 1, axis + 3);
            const float omega_prev = frame(flight_control::kHistoryFrames - 2, axis + 3);
            const float previous_action = frame(flight_control::kHistoryFrames - 1, axis + 6);
            const float error_velocity = error_now - error_prev;
            const float error_curve = error_now - 2.0f * error_prev + error_old;
            const float omega_velocity = omega_now - omega_prev;
            const float value =
                -params_[offset + 0U] * error_now -
                params_[offset + 1U] * omega_now -
                params_[offset + 2U] * error_velocity -
                params_[offset + 3U] * error_curve -
                params_[offset + 4U] * omega_velocity -
                params_[offset + 5U] * previous_action +
                params_[offset + 6U] * error_prev +
                params_[offset + 7U];
            action[axis] = std::tanh(params_[27U] * value + params_[offset + 8U]);
        }
        return action;
    }

private:
    ParameterVector params_{};
};

float average_score(const std::vector<flight_control::EvaluationResult>& results) {
    float total = 0.0f;
    for (const auto& result : results) {
        total += result.metrics.score;
    }
    return results.empty() ? 0.0f : total / static_cast<float>(results.size());
}

SearchResult evaluate(const ParameterVector& params) {
    auto policy = std::make_shared<LinearHistoryPolicy>(params);
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

float objective(const SearchResult& result) {
    return result.average_score + 0.35f * result.worst_score + 8.0f * static_cast<float>(result.stable_count);
}

ParameterVector clamp_params(ParameterVector params) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const std::size_t offset = axis * 9U;
        params[offset + 0U] = std::clamp(params[offset + 0U], 0.1f, 18.0f);
        params[offset + 1U] = std::clamp(params[offset + 1U], 0.01f, 5.5f);
        params[offset + 2U] = std::clamp(params[offset + 2U], -12.0f, 12.0f);
        params[offset + 3U] = std::clamp(params[offset + 3U], -9.0f, 9.0f);
        params[offset + 4U] = std::clamp(params[offset + 4U], -6.0f, 6.0f);
        params[offset + 5U] = std::clamp(params[offset + 5U], -1.4f, 1.4f);
        params[offset + 6U] = std::clamp(params[offset + 6U], -6.0f, 6.0f);
        params[offset + 7U] = std::clamp(params[offset + 7U], -0.5f, 0.5f);
        params[offset + 8U] = std::clamp(params[offset + 8U], -0.5f, 0.5f);
    }
    params[27U] = std::clamp(params[27U], 0.25f, 2.2f);
    return params;
}

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

}  // namespace

int main() {
    std::mt19937 rng(20260626U);
    ParameterVector best_params{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const std::size_t offset = axis * 9U;
        best_params[offset + 0U] = 12.0f;
        best_params[offset + 1U] = 1.2395f;
        best_params[offset + 2U] = -4.11329f;
        best_params[offset + 3U] = -2.78633f;
        best_params[offset + 4U] = 3.88242f;
        best_params[offset + 5U] = -0.715761f;
        best_params[offset + 6U] = -4.0f;
        best_params[offset + 7U] = 0.0106963f;
        best_params[offset + 8U] = 0.0f;
    }
    best_params[27U] = 1.0f;
    SearchResult best = evaluate(best_params);
    print_result("seed", best);

    ParameterVector sigma{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const std::size_t offset = axis * 9U;
        sigma[offset + 0U] = 1.4f;
        sigma[offset + 1U] = 0.28f;
        sigma[offset + 2U] = 0.95f;
        sigma[offset + 3U] = 0.85f;
        sigma[offset + 4U] = 0.75f;
        sigma[offset + 5U] = 0.10f;
        sigma[offset + 6U] = 0.8f;
        sigma[offset + 7U] = 0.035f;
        sigma[offset + 8U] = 0.035f;
    }
    sigma[27U] = 0.10f;

    for (int generation = 0; generation < 180; ++generation) {
        const int population = generation < 70 ? 54 : 34;
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
            }
        }
        for (float& value : sigma) {
            value *= 0.982f;
        }
        if (generation % 25 == 24) {
            print_result("progress", best);
        }
    }

    std::cout << flight_control::format_metrics_table(best.results);
    print_result("final", best);
    return best.stable_count == static_cast<int>(best.results.size()) ? 0 : 1;
}
