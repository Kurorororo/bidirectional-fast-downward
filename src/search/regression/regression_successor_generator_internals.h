#ifndef REGRESSION_SUCCESSOR_GENERATOR_INTERNAL_H
#define REGRESSION_SUCCESSOR_GENERATOR_INTERNAL_H

#include "../abstract_task.h"
#include "../operator_id.h"
#include "../task_utils/successor_generator_internals.h"

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

 public:
  GeneratorSwitchFact(
      const FactPair &fact, int unknown_val,
      std::unique_ptr<successor_generator::GeneratorBase> generator_true,
      std::unique_ptr<successor_generator::GeneratorBase> generator_false);
  virtual void generate_applicable_ops(
      const State &state,
      std::vector<OperatorID> &applicable_ops) const override;
  virtual void generate_applicable_ops(
      const GlobalState &state,
      std::vector<OperatorID> &applicable_ops) const override;
};

}  // namespace regression_successor_generator

#endif