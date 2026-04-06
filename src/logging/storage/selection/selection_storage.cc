#include "selection_storage.h"
#include <iomanip>
#include "instruction.h"

void SelectionStorage::init(const int& seed_tpg, const int& pid,
                            const int& n_task, const int& max_memory_size) {
   std::string filename = generate_filename("selection", seed_tpg, pid);

   file_.open(filename);
   file_ << "generation,phase,task_set";
   for (int i = 0; i < n_task; i++) {
      file_ << ",fitness_task_" << i;
   }
   for (int i = 0; i < n_task; i++) {
      file_ << ",team_complexity_task_" << i;
   }
   for (int i = 0; i < n_task; i++) {
      file_ << ",op_complexity_task_" << i;
   }
   for (int i = 0; i < n_task; i++) {
      file_ << ",flop_complexity_task_" << i;
   }
   file_ << ",team_id,team_size,age,n_outcomes,fitness_value_for_selection,"
            "team_count,"
            "program_count,program_instruction_count,effective_program_"
            "instruction_count";

   appendOperationHeaders();

   for (int i = 2; i <= max_memory_size; i++) {
      file_ << ",m_size_" << i;
   }

   file_ << "\n";
   file_.flush();
}

void SelectionStorage::append(const SelectionMetrics& metrics) {
   file_ << metrics.generation << "," << metrics.phase << "," << "_"
         << metrics.task_set << "_";
   for (auto& m : metrics.fitness_per_task) {
      file_ << "," << std::setprecision(6) << m;
   }
   for (auto& m : metrics.team_complexity_per_task) {
      file_ << "," << std::setprecision(6) << m;
   }
   for (auto& m : metrics.op_complexity_per_task) {
      file_ << "," << std::setprecision(6) << m;
   }
   for (auto& m : metrics.flop_complexity_per_task) {
      file_ << "," << std::fixed << std::setprecision(6) << m;
   }
   file_ << "," << metrics.team_id << "," << metrics.team_size << ","
         << metrics.age << "," << metrics.n_outcomes << ","
         << metrics.fitness_value_for_selection << ","
         << metrics.team_count << "," << metrics.program_count << ","
         << metrics.program_instruction_count << ","
         << metrics.effective_program_instruction_count << ","
         << metrics.operations_use << ","
         << metrics.memory_size_counts << "\n";
   file_.flush();
}

void SelectionStorage::appendOperationHeaders() {
   for (std::string op_name : instruction::op_names_) {
      std::transform(op_name.begin(), op_name.end(), op_name.begin(),
                     ::tolower);
      file_ << "," << op_name;
   }
}