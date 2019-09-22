#include "front_to_front_goal_count_heuristic.h"

#include "../option_parser.h"
#include "../plugin.h"

#include <iostream>
using namespace std;

namespace front_to_front_goal_count_heuristic {
FrontToFrontGoalCountHeuristic::FrontToFrontGoalCountHeuristic(
    const Options &opts)
    : FrontToFrontHeuristic(opts) {
  cout << "Initializing goal count heuristic..." << endl;
}

int FrontToFrontGoalCountHeuristic::compute_heuristic(
    const GlobalState &global_state) {
  const State state = convert_global_state(global_state);
  int unsatisfied_goal_count = 0;

  for (FactProxy goal : goal_state) {
    const VariableProxy var = goal.get_variable();
    if (state[var] != goal) {
      ++unsatisfied_goal_count;
    }
  }
  return unsatisfied_goal_count;
}

static shared_ptr<FrontToFrontHeuristic> _parse(OptionParser &parser) {
  parser.document_synopsis("Goal count heuristic", "");
  parser.document_language_support("action costs", "ignored by design");
  parser.document_language_support("conditional effects", "supported");
  parser.document_language_support("axioms", "supported");
  parser.document_property("admissible", "no");
  parser.document_property("consistent", "no");
  parser.document_property("safe", "yes");
  parser.document_property("preferred operators", "no");

  FrontToFrontHeuristic::add_options_to_parser(parser);
  Options opts = parser.parse();
  if (parser.dry_run())
    return nullptr;
  else
    return make_shared<FrontToFrontGoalCountHeuristic>(opts);
}

static Plugin<FrontToFrontHeuristic> _plugin("front_to_front_goalcount",
                                             _parse);
}  // namespace front_to_front_goal_count_heuristic
