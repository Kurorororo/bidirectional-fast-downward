#include "front_to_front_ff_heuristic.h"

#include "../global_state.h"
#include "../option_parser.h"
#include "../plugin.h"

#include "../task_utils/task_properties.h"

#include <cassert>

using namespace std;

namespace front_to_front_ff_heuristic {
// construction and destruction
FrontToFrontFFHeuristic::FrontToFrontFFHeuristic(const Options &opts)
    : FrontToFrontAdditiveHeuristic(opts),
      relaxed_plan(task_proxy.get_operators().size(), false) {
  cout << "Initializing FF heuristic..." << endl;
}

void FrontToFrontFFHeuristic::mark_preferred_operators_and_relaxed_plan(
    const State &state, PropID goal_id) {
  Proposition *goal = get_proposition(goal_id);
  if (!goal->marked) {  // Only consider each subgoal once.
    goal->marked = true;
    OpID op_id = goal->reached_by;
    if (op_id != NO_OP) {  // We have not yet chained back to a start node.
      UnaryOperator *unary_op = get_operator(op_id);
      for (PropID precond : get_preconditions(op_id)) {
        mark_preferred_operators_and_relaxed_plan(state, precond);
      }
      int operator_no = unary_op->operator_no;
      if (operator_no != -1) {
        // This is not an axiom.
        relaxed_plan[operator_no] = true;
        OperatorProxy op = task_proxy.get_operators()[operator_no];
        set_preferred(op);
      }
    }
  }
}

int FrontToFrontFFHeuristic::compute_heuristic(
    const GlobalState &global_state) {
  State state = convert_global_state(global_state);
  int h_add = compute_add_and_ff(state);
  bool reset_goal = false;
  if (h_add == DEAD_END) {
    if (fall_back_to == INITIAL) {
      State initial_state = task_proxy.get_initial_state();
      h_add = compute_add_and_ff(initial_state);

      if (h_add == DEAD_END) return h_add;

      for (PropID goal_id : goal_propositions)
        mark_preferred_operators_and_relaxed_plan(initial_state, goal_id);
    } else if (fall_back_to == GOAL) {
      set_original_goal();
      h_add = compute_add_and_ff(state);

      if (h_add == DEAD_END) {
        do_set_goal();
        return h_add;
      }

      reset_goal = true;

      for (PropID goal_id : goal_propositions)
        mark_preferred_operators_and_relaxed_plan(state, goal_id);
    }
  } else {
    for (PropID goal_id : goal_propositions)
      mark_preferred_operators_and_relaxed_plan(state, goal_id);
  }

  int h_ff = 0;
  for (size_t op_no = 0; op_no < relaxed_plan.size(); ++op_no) {
    if (relaxed_plan[op_no]) {
      relaxed_plan[op_no] = false;  // Clean up for next computation.
      h_ff += task_proxy.get_operators()[op_no].get_cost();
    }
  }

  if (reset_goal) do_set_goal();

  return h_ff;
}

static shared_ptr<FrontToFrontHeuristic> _parse(OptionParser &parser) {
  parser.document_synopsis("FF heuristic", "");
  parser.document_language_support("action costs", "supported");
  parser.document_language_support("conditional effects", "supported");
  parser.document_language_support(
      "axioms",
      "supported (in the sense that the planner won't complain -- "
      "handling of axioms might be very stupid "
      "and even render the heuristic unsafe)");
  parser.document_property("admissible", "no");
  parser.document_property("consistent", "no");
  parser.document_property("safe", "yes for tasks without axioms");
  parser.document_property("preferred operators", "yes");
  parser.add_option<bool>("cache_initial", "fix initial state", "false");

  vector<string> fall_back_to;
  vector<string> fall_back_to_doc;
  fall_back_to.push_back("NONE");
  fall_back_to_doc.push_back("Do not fall back");
  fall_back_to.push_back("INITIAL");
  fall_back_to_doc.push_back("Use the initail state");
  fall_back_to.push_back("GOAL");
  fall_back_to_doc.push_back("Use the original goal");
  parser.add_enum_option("fall_back_to", fall_back_to,
                         "A state to fall back when dead ends.", "NONE",
                         fall_back_to_doc);

  FrontToFrontHeuristic::add_options_to_parser(parser);
  Options opts = parser.parse();
  if (parser.dry_run())
    return nullptr;
  else
    return make_shared<FrontToFrontFFHeuristic>(opts);
}

static Plugin<FrontToFrontHeuristic> _plugin("front_to_front_ff", _parse);
}  // namespace front_to_front_ff_heuristic
