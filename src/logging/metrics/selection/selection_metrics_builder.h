#ifndef SELECTION_METRICS_BUILDER_H
#define SELECTION_METRICS_BUILDER_H

#include "selection_metrics.h"
#include <string>
#include <vector>

class SelectionMetricsBuilder {
public:
    SelectionMetricsBuilder& with_generation(long generation);
    SelectionMetricsBuilder& with_phase(int phase);
    SelectionMetricsBuilder& with_task_set(std::string task_set);
    SelectionMetricsBuilder& with_fitness_per_task(std::vector<double> fitness_per_task);
    SelectionMetricsBuilder& with_team_complexity_per_task(std::vector<double> team_complexity_per_task);
    SelectionMetricsBuilder& with_op_complexity_per_task(std::vector<double> op_complexity_per_task);
    SelectionMetricsBuilder& with_flop_complexity_per_task(std::vector<double> flop_complexity_per_task);
    SelectionMetricsBuilder& with_team_id(long team_id);    
    SelectionMetricsBuilder& with_team_size(int team_size);
    SelectionMetricsBuilder& with_age(int age);
    SelectionMetricsBuilder& with_n_outcomes(int n_outcomes);
    SelectionMetricsBuilder& with_fitness_value_for_selection(double fitness_value_for_selection);
    SelectionMetricsBuilder& with_team_count(int team_count);
    SelectionMetricsBuilder& with_program_count(int program_count);
    SelectionMetricsBuilder& with_total_program_instructions(int total_program_instructions);
    SelectionMetricsBuilder& with_total_effective_program_instructions(int total_effective_program_instructions);
    SelectionMetricsBuilder& with_operations_use(std::vector<int> operations);
    SelectionMetricsBuilder& with_memory_size_counts(std::vector<int> m_size_counts);

    long get_generation() const;
    int get_phase() const;
    std::string get_task_set() const;
    std::vector<double> get_fitness_per_task() const;
    std::vector<double> get_team_complexity_per_task() const;
    std::vector<double> get_op_complexity_per_task() const;
    std::vector<double> get_flop_complexity_per_task() const;
    long get_team_id() const;
    int get_team_size() const;
    long get_age() const;
    int get_n_outcomes() const;
    double get_fitness_value_for_selection() const;
    int get_team_count() const;
    int get_program_count() const;
    int get_total_program_instructions() const;
    int get_total_effective_program_instructions() const;
    std::string get_operations_use() const;
    std::string get_memory_size_counts() const;

    SelectionMetrics build() const;

private:
    long team_id = 0;
    long generation = 0;
    int phase = 0;
    std::string task_set = "";
    std::vector<double> fitness_per_task;
    std::vector<double> team_complexity_per_task;
    std::vector<double> op_complexity_per_task;
    std::vector<double> flop_complexity_per_task;
    int team_size = 0;
    long age = 0;
    int n_outcomes = 0;
    double fitness_value_for_selection = 0.0;
    int team_count = 0;
    int program_count = 0;
    int program_instruction_count = 0;
    int effective_program_instruction_count = 0;
    std::string operations_use = "";
    std::string memory_size_counts = "";
};


#endif