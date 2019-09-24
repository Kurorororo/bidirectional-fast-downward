#ifndef REGRESSION_SUCCESSOR_GENERATOR_H
#define REGRESSION_SUCCESSOR_GENERATOR_H

#include "../task_utils/successor_generator_internals.h"
#include "regression_task.h"

#include <memory>

class GlobalState;
class State;
class OperatorID;

namespace regression_successor_generator {
class RegressionSuccessorGenerator {
  std::unique_ptr<successor_generator::GeneratorBase> root;

 public:
  explicit RegressionSuccessorGenerator(
      std::shared_ptr<const tasks::RegressionTask> task);

  ~RegressionSuccessorGenerator();

  void generate_applicable_ops(const State &state,
                               std::vector<OperatorID> &applicable_ops) const;
  void generate_applicable_ops(const GlobalState &state,
                               std::vector<OperatorID> &applicable_ops) const;
};

}  // namespace regression_successor_generator

#endif