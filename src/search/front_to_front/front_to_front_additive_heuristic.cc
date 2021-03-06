#include "front_to_front_additive_heuristic.h"

#include "../global_state.h"
#include "../option_parser.h"
#include "../plugin.h"

#include "../task_utils/task_properties.h"

#include <cassert>
#include <vector>

using namespace std;

namespace front_to_front_additive_heuristic {
// construction and destruction
FrontToFrontAdditiveHeuristic::FrontToFrontAdditiveHeuristic(
    const Options &opts)
    : FrontToFrontRelaxationHeuristic(opts),
      did_write_overflow_warning(false),
      cache_initial(opts.get<bool>("cache_initial")),
      fall_back_to(FallBackTo(opts.get_enum("fall_back_to"))) {
  cout << "Initializing additive heuristic..." << endl;

  if (cache_initial) precompute_exploration(task_proxy.get_initial_state());
}

void FrontToFrontAdditiveHeuristic::write_overflow_warning() {
  if (!did_write_overflow_warning) {
    // TODO: Should have a planner-wide warning mechanism to handle
    // things like this.
    cout << "WARNING: overflow on h^add! Costs clamped to " << MAX_COST_VALUE
         << endl;
    cerr << "WARNING: overflow on h^add! Costs clamped to " << MAX_COST_VALUE
         << endl;
    did_write_overflow_warning = true;
  }
}

// heuristic computation
void FrontToFrontAdditiveHeuristic::setup_exploration_queue() {
  queue.clear();

  for (Proposition &prop : propositions) {
    prop.cost = -1;
    prop.marked = false;
  }

  // Deal with operators and axioms without preconditions.
  for (UnaryOperator &op : unary_operators) {
    op.unsatisfied_preconditions = op.num_preconditions;
    op.cost = op.base_cost;  // will be increased by precondition costs

    if (op.unsatisfied_preconditions == 0)
      enqueue_if_necessary(op.effect, op.base_cost, get_op_id(op));
  }
}

void FrontToFrontAdditiveHeuristic::setup_exploration_queue_state(
    const State &state) {
  for (FactProxy fact : state) {
    PropID init_prop = get_prop_id(fact);
    enqueue_if_necessary(init_prop, 0, NO_OP);
  }
}

void FrontToFrontAdditiveHeuristic::relaxed_exploration() {
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
      increase_cost(unary_op->cost, prop_cost);
      --unary_op->unsatisfied_preconditions;
      assert(unary_op->unsatisfied_preconditions >= 0);
      if (unary_op->unsatisfied_preconditions == 0)
        enqueue_if_necessary(unary_op->effect, unary_op->cost, op_id);
    }
  }
}

void FrontToFrontAdditiveHeuristic::mark_preferred_operators(const State &state,
                                                             PropID goal_id) {
  Proposition *goal = get_proposition(goal_id);
  if (!goal->marked) {  // Only consider each subgoal once.
    goal->marked = true;
    OpID op_id = goal->reached_by;
    if (op_id != NO_OP) {  // We have not yet chained back to a start node.
      UnaryOperator *unary_op = get_operator(op_id);
      for (PropID precond : get_preconditions(op_id)) {
        mark_preferred_operators(state, precond);
      }
      int operator_no = unary_op->operator_no;
      if (operator_no != -1) {
        // This is not an axiom.
        OperatorProxy op = task_proxy.get_operators()[operator_no];
        set_preferred(op);
      }
    }
  }
}

void FrontToFrontAdditiveHeuristic::precompute_exploration(const State &state) {
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
      increase_cost(unary_op->cost, prop_cost);
      --unary_op->unsatisfied_preconditions;
      assert(unary_op->unsatisfied_preconditions >= 0);
      if (unary_op->unsatisfied_preconditions == 0)
        enqueue_if_necessary(unary_op->effect, unary_op->cost, op_id);
    }
  }
}

int FrontToFrontAdditiveHeuristic::compute_add_and_ff(const State &state) {
  if (cache_initial) {
    for (Proposition &prop : propositions) prop.marked = false;
  } else {
    setup_exploration_queue();
    setup_exploration_queue_state(state);
    relaxed_exploration();
  }

  int total_cost = 0;
  for (PropID goal_id : goal_propositions) {
    const Proposition *goal = get_proposition(goal_id);
    int goal_cost = goal->cost;
    if (goal_cost == -1) return DEAD_END;
    increase_cost(total_cost, goal_cost);
  }
  return total_cost;
}

int FrontToFrontAdditiveHeuristic::compute_heuristic(const State &state) {
  int h = compute_add_and_ff(state);
  if (h != DEAD_END) {
    for (PropID goal_id : goal_propositions)
      mark_preferred_operators(state, goal_id);
  } else if (fall_back_to == INITIAL) {
    State initial_state = task_proxy.get_initial_state();
    h = compute_add_and_ff(initial_state);

    if (h != DEAD_END)
      for (PropID goal_id : goal_propositions)
        mark_preferred_operators(initial_state, goal_id);
  } else if (fall_back_to == GOAL) {
    set_original_goal();
    h = compute_add_and_ff(state);

    if (h != DEAD_END)
      for (PropID goal_id : goal_propositions)
        mark_preferred_operators(state, goal_id);

    do_set_goal();
  }
  return h;
}

int FrontToFrontAdditiveHeuristic::compute_heuristic(
    const GlobalState &global_state) {
  return compute_heuristic(convert_global_state(global_state));
}

void FrontToFrontAdditiveHeuristic::compute_heuristic_for_cegar(
    const State &state) {
  compute_heuristic(state);
}

static shared_ptr<FrontToFrontHeuristic> _parse(OptionParser &parser) {
  parser.document_synopsis("Additive heuristic", "");
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
    return make_shared<FrontToFrontAdditiveHeuristic>(opts);
}

static Plugin<FrontToFrontHeuristic> _plugin("front_to_front_add", _parse);
}  // namespace front_to_front_additive_heuristic
