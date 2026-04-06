#include "selection_metrics_builder.h"
#include "selection_metrics.h"
#include <string>
#include <sstream>
#include <iostream>

SelectionMetricsBuilder& SelectionMetricsBuilder::with_generation(long generation) {
    this->generation = generation;
    return *this;
}

SelectionMetricsBuilder& SelectionMetricsBuilder::with_fitness_per_task(std::vector<double> fitness_per_task) {
    this->fitness_per_task = fitness_per_task;
    return *this;
}

SelectionMetricsBuilder& SelectionMetricsBuilder::with_team_complexity_per_task(std::vector<double> team_complexity_per_task) {
    this->team_complexity_per_task = team_complexity_per_task;
    return *this;
}

SelectionMetricsBuilder& SelectionMetricsBuilder::with_op_complexity_per_task(std::vector<double> op_complexity_per_task) {
    this->op_complexity_per_task = op_complexity_per_task;
    return *this;
}

SelectionMetricsBuilder& SelectionMetricsBuilder::with_flop_complexity_per_task(std::vector<double> flop_complexity_per_task) {
    this->flop_complexity_per_task = flop_complexity_per_task;
    return *this;
}

SelectionMetricsBuilder& SelectionMetricsBuilder::with_task_set(std::string task_set) {
    this->task_set = task_set;
    return *this;
}

SelectionMetricsBuilder& SelectionMetricsBuilder::with_phase(int phase) {
    this->phase = phase;
    return *this;
}

SelectionMetricsBuilder& SelectionMetricsBuilder::with_team_id(long team_id) {
    this->team_id = team_id;
    return *this;
}

SelectionMetricsBuilder& SelectionMetricsBuilder::with_team_size(int team_size) {
    this->team_size = team_size;
    return *this;
}

SelectionMetricsBuilder& SelectionMetricsBuilder::with_age(int age) {
    this->age = age;
    return *this;
}

SelectionMetricsBuilder& SelectionMetricsBuilder::with_n_outcomes(int n_outcomes) {
    this->n_outcomes = n_outcomes;
    return *this;
}

SelectionMetricsBuilder& SelectionMetricsBuilder::with_fitness_value_for_selection(double fitness_value_for_selection) {
    this->fitness_value_for_selection = fitness_value_for_selection;
    return *this;
}

SelectionMetricsBuilder& SelectionMetricsBuilder::with_team_count(int team_count) {
    this->team_count = team_count;
    return *this;
}

SelectionMetricsBuilder& SelectionMetricsBuilder::with_program_count(int program_count) {
    this->program_count = program_count;
    return *this;
}

SelectionMetricsBuilder& SelectionMetricsBuilder::with_total_program_instructions(int total_program_instructions) {
    this->program_instruction_count = total_program_instructions;
    return *this;
}

SelectionMetricsBuilder& SelectionMetricsBuilder::with_total_effective_program_instructions(int total_effective_program_instructions) {
    this->effective_program_instruction_count = total_effective_program_instructions;
    return *this;
}

SelectionMetricsBuilder& SelectionMetricsBuilder::with_operations_use(std::vector<int> operations_use) {
    std::stringstream ss;

    bool first = true;
    for (int op : operations_use) {
        if (!first) {
            ss << ",";
        }
        ss << op;
        first = false;
    }

    this->operations_use = ss.str();
    return *this;
}

SelectionMetricsBuilder& SelectionMetricsBuilder::with_memory_size_counts(std::vector<int> m_size_counts) {
    std::stringstream ss;

    bool first = true;
    for (int c : m_size_counts) {
        if (!first) {
            ss << ",";
        }
        ss << c;
        first = false;
    }

    this->memory_size_counts = ss.str();
    return *this;
}

SelectionMetrics SelectionMetricsBuilder::build() const {
    return { *this };
}

long SelectionMetricsBuilder::get_generation() const {
    return generation;
}

std::vector<double> SelectionMetricsBuilder::get_fitness_per_task() const {
    return fitness_per_task;
}

std::vector<double> SelectionMetricsBuilder::get_team_complexity_per_task() const {
    return team_complexity_per_task;
}

std::vector<double> SelectionMetricsBuilder::get_op_complexity_per_task() const {
    return op_complexity_per_task;
}

std::vector<double> SelectionMetricsBuilder::get_flop_complexity_per_task() const {
    return flop_complexity_per_task;
}

int SelectionMetricsBuilder::get_phase() const {
    return phase;
}

std::string SelectionMetricsBuilder::get_task_set() const {
    return task_set;
}

long SelectionMetricsBuilder::get_team_id() const {
    return team_id;
}

int SelectionMetricsBuilder::get_team_size() const {
    return team_size;
}

long SelectionMetricsBuilder::get_age() const {
    return age;
}

int SelectionMetricsBuilder::get_n_outcomes() const {
    return n_outcomes;
}

double SelectionMetricsBuilder::get_fitness_value_for_selection() const {
    return fitness_value_for_selection;
}

int SelectionMetricsBuilder::get_team_count() const {
    return team_count;
}

int SelectionMetricsBuilder::get_program_count() const {
    return program_count;
}

int SelectionMetricsBuilder::get_total_program_instructions() const {
    return program_instruction_count;
}

int SelectionMetricsBuilder::get_total_effective_program_instructions() const {
    return effective_program_instruction_count;
}

std::string SelectionMetricsBuilder::get_operations_use() const {
    return operations_use;
}

std::string SelectionMetricsBuilder::get_memory_size_counts() const {
    return memory_size_counts;
}
