#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

#include "flight_control/eval/simulation_runner.hpp"
#include "flight_control/model/generated_policy.hpp"
#include "flight_control/model/static_mlp_policy.hpp"

namespace {

void check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

}  // namespace

int main() {
    auto policy = std::make_shared<flight_control::StaticMlpPolicy>(flight_control::make_generated_policy_weights());
    flight_control::SimulationRunner runner(policy);
    const auto scenarios = flight_control::default_evaluation_scenarios(flight_control::generated_model_config());

    std::vector<flight_control::SimulationResult> results;
    for (const auto& scenario : scenarios) {
        results.push_back(runner.run(scenario));
    }

    std::cout << flight_control::format_metrics_table(results);
    float total_score = 0.0f;
    for (const auto& result : results) {
        check(result.metrics.stable, result.metrics.scenario_name.c_str());
        check(result.metrics.score >= 70.0f, "closed-loop score should stay above scenario regression floor");
        total_score += result.metrics.score;
    }
    check(total_score / static_cast<float>(results.size()) >= 80.0f,
          "closed-loop average score should stay above system regression floor");

    std::cout << "flight_control_system_tests passed\n";
    return 0;
}
