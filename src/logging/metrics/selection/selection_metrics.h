#ifndef SELECTION_METRICS_H
#define SELECTION_METRICS_H

#include <vector>
#include <optional>
#include <string>

class SelectionMetricsBuilder;

struct SelectionMetrics {
    const long team_id;
    const long generation;
    const int phase;
    const std::string task_set;
    std::vector<double> fitness_per_task;
    std::vector<double> team_complexity_per_task;
    std::vector<double> op_complexity_per_task;
    std::vector<double> flop_complexity_per_task;
    const int team_size;
    const long age;
    const int n_outcomes;
    const double fitness_value_for_selection;
    const int team_count;
    const int program_count;
    const int program_instruction_count;
    const int effective_program_instruction_count;
    const std::string operations_use;
    const std::string memory_size_counts;


    SelectionMetrics(const SelectionMetricsBuilder& builder);
};

#include "selection_metrics_builder.h"

#endif
