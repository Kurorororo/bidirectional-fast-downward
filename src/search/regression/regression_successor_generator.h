#ifndef REGRESSION_SUCCESSOR_GENERATOR_H
#define REGRESSION_SUCCESSOR_GENERATOR_H

#include "../operator_id.h"
#include "../task_utils/successor_generator_internals.h"
#include "regression_task.h"

#include <memory>

class GlobalState;
class State;

namespace regression_successor_generator {
class GeneratorSwitchFact : public successor_generator::GeneratorBase {
  int var;
  int val;
  int unknown_val;
  std::unique_ptr<successor_generator::GeneratorBase> generator_true;
  std::unique_ptr<successor_generator::GeneratorBase> generator_false;
  std::unique_ptr<successor_generator::GeneratorBase> generator_dont_care;

 public:
  GeneratorSwitchFact(
      const FactPair &fact, int unknown_val,
      std::unique_ptr<successor_generator::GeneratorBase> generator_true,
      std::unique_ptr<successor_generator::GeneratorBase> generator_false,
      std::unique_ptr<successor_generator::GeneratorBase> generator_dont_care);
  virtual void generate_applicable_ops(
      const State &state,
      std::vector<OperatorID> &applicable_ops) const override;
  virtual void generate_applicable_ops(
      const GlobalState &state,
      std::vector<OperatorID> &applicable_ops) const override;
};

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