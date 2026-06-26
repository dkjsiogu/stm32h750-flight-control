#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "flight_control/eval/closed_loop_evaluator.hpp"
#include "flight_control/model/generated_policy.hpp"
#include "flight_control/model/static_mlp_policy.hpp"

int main() {
    using namespace flight_control;

    auto policy = std::make_shared<StaticMlpPolicy>(make_generated_policy_weights());
    ModelAdapterConfig model_config = generated_model_config();
    ClosedLoopEvaluator runner(policy);

    std::vector<EvaluationResult> results;
    for (const EvaluationScenario& scenario : default_evaluation_scenarios(model_config)) {
        results.push_back(runner.run(scenario));
    }

    const std::string table = format_metrics_table(results);
    const std::string report = format_markdown_report(results);

    std::filesystem::create_directories("output");
    std::ofstream report_file("output/flight_evaluation_report.md");
    report_file << report;
    std::ofstream csv_file("output/flight_evaluation_metrics.csv");
    csv_file << table;

    std::cout << table;
    std::cout << "report=output/flight_evaluation_report.md\n";
    return 0;
}
