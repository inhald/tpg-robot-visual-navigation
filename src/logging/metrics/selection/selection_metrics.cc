#include "selection_metrics.h"
#include "selection_metrics_builder.h"

SelectionMetrics::SelectionMetrics(const SelectionMetricsBuilder& builder)
    : team_id(builder.get_team_id()),
      generation(builder.get_generation()),
      phase(builder.get_phase()),
      task_set(builder.get_task_set()),
      fitness_per_task(builder.get_fitness_per_task()),
      team_complexity_per_task(builder.get_team_complexity_per_task()),
      op_complexity_per_task(builder.get_op_complexity_per_task()),
      flop_complexity_per_task(builder.get_flop_complexity_per_task()),
      team_size(builder.get_team_size()),
      age(builder.get_age()),
      n_outcomes(builder.get_n_outcomes()),
      fitness_value_for_selection(builder.get_fitness_value_for_selection()),
      team_count(builder.get_team_count()),
      program_count(builder.get_program_count()),
      program_instruction_count(builder.get_total_program_instructions()),
      effective_program_instruction_count(builder.get_total_effective_program_instructions()),
      operations_use(builder.get_operations_use()),
      memory_size_counts(builder.get_memory_size_counts()) {
}