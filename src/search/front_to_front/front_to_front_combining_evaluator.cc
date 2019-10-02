#include "front_to_front_combining_evaluator.h"

#include "../evaluation_context.h"
#include "../evaluation_result.h"

using namespace std;

namespace front_to_front_combining_evaluator {
FrontToFrontCombiningEvaluator::FrontToFrontCombiningEvaluator(
    const vector<shared_ptr<FrontToFrontHeuristic>> &subevaluators_)
    : subevaluators(subevaluators_) {
  all_dead_ends_are_reliable = true;
  for (const shared_ptr<Evaluator> &subevaluator : subevaluators)
    if (!subevaluator->dead_ends_are_reliable())
      all_dead_ends_are_reliable = false;
}

FrontToFrontCombiningEvaluator::~FrontToFrontCombiningEvaluator() {}

bool FrontToFrontCombiningEvaluator::dead_ends_are_reliable() const {
  return all_dead_ends_are_reliable;
}

EvaluationResult FrontToFrontCombiningEvaluator::compute_result(
    EvaluationContext &eval_context) {
  // This marks no preferred operators.
  EvaluationResult result;
  vector<int> values;
  values.reserve(subevaluators.size());

  // Collect component values. Return infinity if any is infinite.
  for (const shared_ptr<Evaluator> &subevaluator : subevaluators) {
    int value =
        eval_context.get_evaluator_value_or_infinity(subevaluator.get());
    if (value == EvaluationResult::INFTY) {
      result.set_evaluator_value(value);
      return result;
    } else {
      values.push_back(value);
    }
  }

  // If we arrived here, all subevaluator values are finite.
  result.set_evaluator_value(combine_values(values));
  return result;
}

void FrontToFrontCombiningEvaluator::get_path_dependent_evaluators(
    set<Evaluator *> &evals) {
  for (auto &subevaluator : subevaluators)
    subevaluator->get_path_dependent_evaluators(evals);
}

void FrontToFrontCombiningEvaluator::set_goal(const GlobalState &state) {
  for (auto &subevaluator : subevaluators) subevaluator->set_goal(state);
}

}  // namespace front_to_front_combining_evaluator
