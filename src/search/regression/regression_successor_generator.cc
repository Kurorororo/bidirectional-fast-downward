#include "regression_successor_generator.h"

#include "../global_state.h"
#include "../task_proxy.h"

#include <cassert>

using namespace std;

namespace regression_successor_generator {

GeneratorSwitchFact::GeneratorSwitchFact(
    const FactPair &fact, int unknown_val,
    std::unique_ptr<successor_generator::GeneratorBase> generator_true,
    std::unique_ptr<successor_generator::GeneratorBase> generator_false,
    std::unique_ptr<successor_generator::GeneratorBase> generator_dont_care)
    : var(fact.var),
      val(fact.value),
      unknown_val(unknown_val),
      generator_true(move(generator_true)),
      generator_false(move(generator_false)),
      generator_dont_care(move(generator_dont_care)) {
  assert(this->generator_true);
  assert(this->generator_false);
  assert(this->generator_dont_care);
}

void GeneratorSwitchFact::generate_applicable_ops(
    const State &state, vector<OperatorID> &applicable_ops) const {
  if (state[var].get_value() == unknown_val) {
    generator_true->generate_applicable_ops(state, applicable_ops);
    generator_false->generate_applicable_ops(state, applicable_ops);
  } else if (state[var].get_value() == val) {
    generator_true->generate_applicable_ops(state, applicable_ops);
  } else {
    generator_false->generate_applicable_ops(state, applicable_ops);
  }

  generator_dont_care->generate_applicable_ops(state, applicable_ops);
}

void GeneratorSwitchFact::generate_applicable_ops(
    const GlobalState &state, vector<OperatorID> &applicable_ops) const {
  if (state[var] == unknown_val) {
    generator_true->generate_applicable_ops(state, applicable_ops);
    generator_false->generate_applicable_ops(state, applicable_ops);
  } else if (state[var] == val) {
    generator_true->generate_applicable_ops(state, applicable_ops);
  } else {
    generator_false->generate_applicable_ops(state, applicable_ops);
  }

  generator_dont_care->generate_applicable_ops(state, applicable_ops);
}

}  // namespace regression_successor_generator
