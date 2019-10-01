#include "front_to_front_greater.h"

#include "../evaluation_context.h"
#include "../evaluation_result.h"
#include "../option_parser.h"
#include "../plugin.h"

#include <cstdlib>
#include <sstream>

using namespace std;

namespace front_to_front_greater {
const int FrontToFrontGreater::INFTY = numeric_limits<int>::max() - 1;

FrontToFrontGreater::FrontToFrontGreater(const Options &opts)
    : FrontToFrontHeuristic(opts),
      evaluator(opts.get<shared_ptr<FrontToFrontHeuristic>>("eval")) {}

FrontToFrontGreater::~FrontToFrontGreater() {}

EvaluationResult FrontToFrontGreater::compute_result(
    EvaluationContext &eval_context) {
  EvaluationResult result = evaluator->compute_result(eval_context);

  if (result.is_infinite()) return result;

  result.set_evaluator_value(INFTY - result.get_evaluator_value());

  return result;
}

void FrontToFrontGreater::set_goal(const GlobalState &state) {
  evaluator->set_goal(state);
}

bool FrontToFrontGreater::dead_ends_are_reliable() const {
  return evaluator->dead_ends_are_reliable();
}

void FrontToFrontGreater::get_path_dependent_evaluators(
    set<Evaluator *> &evals) {
  evaluator->get_path_dependent_evaluators(evals);
}

static shared_ptr<FrontToFrontHeuristic> _parse(OptionParser &parser) {
  parser.document_synopsis(
      "Weighted evaluator",
      "Multiplies the value of the evaluator with the given weight.");
  parser.add_option<shared_ptr<FrontToFrontHeuristic>>("eval", "evaluator");
  FrontToFrontHeuristic::add_options_to_parser(parser);
  Options opts = parser.parse();
  if (parser.dry_run())
    return nullptr;
  else
    return make_shared<FrontToFrontGreater>(opts);
}

static Plugin<FrontToFrontHeuristic> _plugin("front_to_front_greater", _parse);
}  // namespace front_to_front_greater
