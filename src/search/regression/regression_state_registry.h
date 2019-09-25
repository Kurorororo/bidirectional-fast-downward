#ifndef REGRESSION_STATE_REGISTRY_H
#define REGRESSION_STATE_REGISTRY_H

#include "../abstract_task.h"
#include "../axioms.h"
#include "../global_state.h"
#include "../state_id.h"
#include "../state_registry.h"
#include "../task_proxy.h"

#include "../algorithms/int_hash_set.h"
#include "../algorithms/int_packer.h"
#include "../algorithms/segmented_vector.h"
#include "../algorithms/subscriber.h"
#include "../utils/hash.h"

#include <set>
#include <utility>
#include <vector>

class RegressionStateRegistry : public StateRegistry {
  std::vector<std::vector<std::vector<std::pair<int, int>>>> fact_to_mutexes;

  void init_mutex();

 public:
  explicit RegressionStateRegistry(const TaskProxy &task_proxy);
  ~RegressionStateRegistry();

  StateID get_predecessor_state(const GlobalState &predecessor,
                                const OperatorProxy &op);
};

#endif
