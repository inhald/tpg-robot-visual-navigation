#ifndef evaluators_gazebo_h
#define evaluators_gazebo_h


#include "cstring"
#include "cstdio"
#include "cstdlib"
#include "thread"
#include "unistd.h"

#include "EvalData.h"
#include "GazeboEnv.h"
#include "ActionWrappers.h"
#include "TPG.h"
#include "filesystem"
#include "misc.h"




inline void EvalGazebo(TPG& tpg, EvalData& eval_data) {
  GazeboEnv* task = dynamic_cast<GazeboEnv*>(eval_data.task);
  task->Reset(tpg.rngs_[AUX_SEED]);

  eval_data.n_prediction = 0;
  eval_data.obs = new state (task->GetObsSize());
  eval_data.obs->Set(task->GetObsVec(eval_data.partially_observable));



  while (!task->Terminal()) {
    tpg.GetAction(eval_data);  
    auto ctrl = WrapDiscreteActionGazebo(eval_data);




    std::vector<double> action = (ctrl == 0) ? std::vector<double>{0.46,0.0} :
				 (ctrl == 1) ? std::vector<double>{0.05, 1.20} : 
				 (ctrl == 2) ? std::vector<double>{0.05, -1.20} :
				 std::vector<double>{0.0, 0.0};


    TaskEnv::Results r = task->SimStep(action);

    eval_data.stats_double[REWARD1_IDX] += r.r1;  
    eval_data.AccumulateStepData();
    eval_data.obs->Set(task->GetObsVec(eval_data.partially_observable));


  }


  delete eval_data.obs;
}




#endif 
