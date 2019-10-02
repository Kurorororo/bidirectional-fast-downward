#include "front_to_front_sum_evaluator.h"

#include "../option_parser.h"
#include "../plugin.h"

#include <cassert>
#include <limits>

using namespace std;

namespace front_to_front_sum_evaluator {
FrontToFrontSumEvaluator::FrontToFrontSumEvaluator(const Options &opts)
    : FrontToFrontCombiningEvaluator(
          opts.get_list<shared_ptr<FrontToFrontHeuristic>>("evals")) {}

FrontToFrontSumEvaluator::FrontToFrontSumEvaluator(
    const vector<shared_ptr<FrontToFrontHeuristic>> &evals)
    : FrontToFrontCombiningEvaluator(evals) {}

FrontToFrontSumEvaluator::~FrontToFrontSumEvaluator() {}

int FrontToFrontSumEvaluator::combine_values(const vector<int> &values) {
  int result = 0;
  for (int value : values) {
    assert(value >= 0);
    result += value;
    assert(result >= 0);  // Check against overflow.
  }
  return result;
}

static shared_ptr<FrontToFrontHeuristic> _parse(OptionParser &parser) {
  parser.document_synopsis("Sum evaluator",
                           "Calculates the sum of the sub-evaluators.");

  parser.add_list_option<shared_ptr<FrontToFrontHeuristic>>(
      "evals", "at least one evaluator");
  FrontToFrontHeuristic::add_options_to_parser(parser);
  Options opts = parser.parse();

  opts.verify_list_non_empty<shared_ptr<FrontToFrontHeuristic>>("evals");

  if (parser.dry_run())
    return nullptr;
  else
    return make_shared<FrontToFrontSumEvaluator>(opts);
}

static Plugin<FrontToFrontHeuristic> _plugin("front_to_front_sum", _parse);
}  // namespace front_to_front_sum_evaluator
