#include "regression_successor_generator_internals.h"

#include "../global_state.h"
#include "../task_proxy.h"

#include <cassert>

using namespace std;

namespace regression_successor_generator {

GeneratorSwitchFact::GeneratorSwitchFact(
    const FactPair &fact, int unknown_val,
    std::unique_ptr<successor_generator::GeneratorBase> generator_true,
    std::unique_ptr<successor_generator::GeneratorBase> generator_false)
    : var(fact.var),
      val(fact.value),
      unknown_val(unknown_val),
      generator_true(move(generator_true)),
      generator_false(move(generator_false)) {}

void GeneratorSwitchFact::generate_applicable_ops(
    const State &state, vector<OperatorID> &applicable_ops) const {
  if (state[var].get_value() == unknown_val) {
    if (generator_true != nullptr)
      generator_true->generate_applicable_ops(state, applicable_ops);
    if (generator_false != nullptr)
      generator_false->generate_applicable_ops(state, applicable_ops);
  } else if (state[var].get_value() == val) {
    if (generator_true)
      if (generator_true != nullptr)
        generator_true->generate_applicable_ops(state, applicable_ops);
  } else {
    if (generator_false != nullptr)
      generator_false->generate_applicable_ops(state, applicable_ops);
  }
}

void GeneratorSwitchFact::generate_applicable_ops(
    const GlobalState &state, vector<OperatorID> &applicable_ops) const {
  if (state[var] == unknown_val) {
    if (generator_true != nullptr) {
      generator_true->generate_applicable_ops(state, applicable_ops);
    }
    if (generator_false != nullptr) {
      generator_false->generate_applicable_ops(state, applicable_ops);
    }
  } else if (state[var] == val) {
    if (generator_true != nullptr) {
      generator_true->generate_applicable_ops(state, applicable_ops);
    }
  } else {
    if (generator_false != nullptr) {
      generator_false->generate_applicable_ops(state, applicable_ops);
    }
  }
}

}  // namespace regression_successor_generator
