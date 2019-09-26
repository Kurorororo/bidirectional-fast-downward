#include "front_to_front_heuristic.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "../tasks/cost_adapted_task.h"

using namespace std;

FrontToFrontHeuristic::FrontToFrontHeuristic(const Options &opts)
    : Evaluator(opts.get_unparsed_config(), true, true, true),
      regression(opts.get<bool>("partial_state")),
      task(opts.get<shared_ptr<AbstractTask>>("transform")),
      task_proxy(*task) {}

FrontToFrontHeuristic::~FrontToFrontHeuristic() {}

void FrontToFrontHeuristic::set_preferred(const OperatorProxy &op) {
  preferred_operators.insert(op.get_ancestor_operator_id(task.get()));
}

State FrontToFrontHeuristic::convert_global_state(
    const GlobalState &global_state) const {
  return task_proxy.convert_ancestor_state(global_state.unpack());
}

void FrontToFrontHeuristic::set_goal(const GlobalState &global_state) {
  auto goal_state = task_proxy.convert_ancestor_state(global_state.unpack());
  current_goal.clear();

  for (auto f : goal_state) {
    VariableProxy var = f.get_variable();
    int value = f.get_value();

    if (!partial_state || value < var.get_domain_size() - 1)
      current_goal.emplace_back(
          make_pair(f.get_variable().get_id(), f.get_value()));
  }
}

void FrontToFrontHeuristic::add_options_to_parser(OptionParser &parser) {
  parser.add_option<shared_ptr<AbstractTask>>(
      "transform",
      "Optional task transformation for the heuristic."
      " Currently, adapt_costs() and no_transform() are available.",
      "no_transform()");
  parser.add_option<bool>("partial_state", "evaluate partial state", "false");
}

EvaluationResult FrontToFrontHeuristic::compute_result(
    EvaluationContext &eval_context) {
  EvaluationResult result;

  assert(preferred_operators.empty());

  const GlobalState &state = eval_context.get_state();
  int heuristic = compute_heuristic(state);
  result.set_count_evaluation(true);

  assert(heuristic == DEAD_END || heuristic >= 0);

  if (heuristic == DEAD_END) {
    /*
      It is permissible to mark preferred operators for dead-end
      states (thus allowing a heuristic to mark them on-the-fly
      before knowing the final result), but if it turns out we
      have a dead end, we don't want to actually report any
      preferred operators.
    */
    preferred_operators.clear();
    heuristic = EvaluationResult::INFTY;
  }

  result.set_evaluator_value(heuristic);
  result.set_preferred_operators(preferred_operators.pop_as_vector());
  assert(preferred_operators.empty());

  return result;
}

static PluginTypePlugin<FrontToFrontHeuristic> _type_plugin(
    "FrontToFrontHeuristic", "front to front heuristic functions",
    "front_to_front_evaluator", "front_to_front_heuristic");
