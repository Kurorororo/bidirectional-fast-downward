#include "front_to_front_g_evaluator.h"

#include "../evaluation_context.h"
#include "../evaluation_result.h"
#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace front_to_front_g_evaluator {
EvaluationResult FrontToFrontGEvaluator::compute_result(
    EvaluationContext &eval_context) {
  EvaluationResult result;
  result.set_evaluator_value(eval_context.get_g_value());
  return result;
}

static shared_ptr<FrontToFrontHeuristic> _parse(OptionParser &parser) {
  parser.document_synopsis(
      "g-value evaluator",
      "Returns the g-value (path cost) of the search node.");
  FrontToFrontHeuristic::add_options_to_parser(parser);
  Options opts = parser.parse();
  if (parser.dry_run())
    return nullptr;
  else
    return make_shared<FrontToFrontGEvaluator>(opts);
}

static Plugin<FrontToFrontHeuristic> _plugin("front_to_front_g", _parse);
}  // namespace front_to_front_g_evaluator
