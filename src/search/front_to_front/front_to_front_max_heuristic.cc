#include "front_to_front_max_heuristic.h"

#include "../global_state.h"
#include "../option_parser.h"
#include "../plugin.h"

#include <cassert>
#include <vector>

using namespace std;

namespace front_to_front_max_heuristic {
/*
  TODO: At the time of this writing, this shares huge amounts of code
        with h^add, and the two should be refactored so that the
        common code is only included once, in so far as this is
        possible without sacrificing run-time. We may want to avoid
        virtual calls in the inner-most loops; maybe a templated
        strategy pattern is an option. Right now, the only differences
        to the h^add code are the use of max() instead of add() and
        the lack of preferred operator support (but we might actually
        reintroduce that if it doesn't hurt performance too much).
 */

// construction and destruction
FrontToFrontHSPMaxHeuristic::FrontToFrontHSPMaxHeuristic(const Options &opts)
    : FrontToFrontRelaxationHeuristic(opts),
      cache_initial(opts.get<bool>("cache_initial")),
      fall_back_to(FallBackTo(opts.get_enum("fall_back_to"))) {
  cout << "Initializing HSP max heuristic..." << endl;

  if (cache_initial) precompute_exploration(task_proxy.get_initial_state());
}

// heuristic computation
void FrontToFrontHSPMaxHeuristic::setup_exploration_queue() {
  queue.clear();

  for (Proposition &prop : propositions) prop.cost = -1;

  // Deal with operators and axioms without preconditions.
  for (UnaryOperator &op : unary_operators) {
    op.unsatisfied_preconditions = op.num_preconditions;
    op.cost = op.base_cost;  // will be increased by precondition costs

    if (op.unsatisfied_preconditions == 0)
      enqueue_if_necessary(op.effect, op.base_cost);
  }
}

void FrontToFrontHSPMaxHeuristic::setup_exploration_queue_state(
    const State &state) {
  for (FactProxy fact : state) {
    PropID init_prop = get_prop_id(fact);
    enqueue_if_necessary(init_prop, 0);
  }
}

void FrontToFrontHSPMaxHeuristic::relaxed_exploration() {
  int unsolved_goals = goal_propositions.size();
  while (!queue.empty()) {
    pair<int, PropID> top_pair = queue.pop();
    int distance = top_pair.first;
    PropID prop_id = top_pair.second;
    Proposition *prop = get_proposition(prop_id);
    int prop_cost = prop->cost;
    assert(prop_cost >= 0);
    assert(prop_cost <= distance);
    if (prop_cost < distance) continue;
    if (prop->is_goal && --unsolved_goals == 0) return;
    for (OpID op_id : precondition_of_pool.get_slice(
             prop->precondition_of, prop->num_precondition_occurences)) {
      UnaryOperator *unary_op = get_operator(op_id);
      unary_op->cost = max(unary_op->cost, unary_op->base_cost + prop_cost);
      --unary_op->unsatisfied_preconditions;
      assert(unary_op->unsatisfied_preconditions >= 0);
      if (unary_op->unsatisfied_preconditions == 0)
        enqueue_if_necessary(unary_op->effect, unary_op->cost);
    }
  }
}

void FrontToFrontHSPMaxHeuristic::precompute_exploration(const State &state) {
  setup_exploration_queue();
  setup_exploration_queue_state(state);

  while (!queue.empty()) {
    pair<int, PropID> top_pair = queue.pop();
    int distance = top_pair.first;
    PropID prop_id = top_pair.second;
    Proposition *prop = get_proposition(prop_id);
    int prop_cost = prop->cost;
    assert(prop_cost >= 0);
    assert(prop_cost <= distance);
    if (prop_cost < distance) continue;
    for (OpID op_id : precondition_of_pool.get_slice(
             prop->precondition_of, prop->num_precondition_occurences)) {
      UnaryOperator *unary_op = get_operator(op_id);
      unary_op->cost = max(unary_op->cost, unary_op->base_cost + prop_cost);
      --unary_op->unsatisfied_preconditions;
      assert(unary_op->unsatisfied_preconditions >= 0);
      if (unary_op->unsatisfied_preconditions == 0)
        enqueue_if_necessary(unary_op->effect, unary_op->cost);
    }
  }
}

int FrontToFrontHSPMaxHeuristic::compute_heuristic(
    const GlobalState &global_state) {
  if (cache_initial) {
    for (Proposition &prop : propositions) prop.marked = false;
  } else {
    const State state = convert_global_state(global_state);
    setup_exploration_queue();
    setup_exploration_queue_state(state);
    relaxed_exploration();
  }

  int total_cost = 0;
  for (PropID goal_id : goal_propositions) {
    const Proposition *goal = get_proposition(goal_id);
    int goal_cost = goal->cost;
    if (goal_cost == -1) return DEAD_END;
    total_cost = max(total_cost, goal_cost);
  }
  return total_cost;
}

static shared_ptr<FrontToFrontHeuristic> _parse(OptionParser &parser) {
  parser.document_synopsis("Max heuristic", "");
  parser.document_language_support("action costs", "supported");
  parser.document_language_support("conditional effects", "supported");
  parser.document_language_support(
      "axioms",
      "supported (in the sense that the planner won't complain -- "
      "handling of axioms might be very stupid "
      "and even render the heuristic unsafe)");
  parser.document_property("admissible", "yes for tasks without axioms");
  parser.document_property("consistent", "yes for tasks without axioms");
  parser.document_property("safe", "yes for tasks without axioms");
  parser.document_property("preferred operators", "no");
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
    return make_shared<FrontToFrontHSPMaxHeuristic>(opts);
}

static Plugin<FrontToFrontHeuristic> _plugin("front_to_front_hmax", _parse);
}  // namespace front_to_front_max_heuristic
