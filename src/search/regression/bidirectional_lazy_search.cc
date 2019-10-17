#include "bidirectional_lazy_search.h"

#include "../front_to_front/front_to_front_open_list_factory.h"
#include "../option_parser.h"

#include "../algorithms/ordered_set.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"
#include "../utils/rng.h"
#include "../utils/rng_options.h"

#include <algorithm>
#include <limits>
#include <vector>

using namespace std;

namespace bidirectional_lazy_search {
BidirectionalLazySearch::BidirectionalLazySearch(const Options &opts)
    : SearchEngine(opts),
      for_open_list(opts.get<shared_ptr<FrontToFrontOpenListFactory>>("open_f")
                        ->create_edge_open_list()),
      bac_open_list(opts.get<shared_ptr<FrontToFrontOpenListFactory>>("open_b")
                        ->create_edge_open_list()),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      randomize_successors(opts.get<bool>("randomize_successors")),
      preferred_successors_first(opts.get<bool>("preferred_successors_first")),
      prune_goal(opts.get<bool>("prune_goal")),
      rng(utils::parse_rng_from_options(opts)),
      partial_state_task(tasks::PartialStateTask::get_partial_state_task()),
      partial_state_task_proxy(*partial_state_task),
      regression_state_registry(partial_state_task_proxy),
      partial_state_search_space(regression_state_registry),
      regression_task(tasks::RegressionTask::get_regression_task()),
      regression_task_proxy(*regression_task),
      regression_successor_generator(regression_task),
      current_direction(FORWARD),
      directions(NONE),
      for_current_state(regression_state_registry.get_initial_state()),
      for_current_predecessor_id(StateID::no_state),
      for_current_operator_id(OperatorID::no_operator),
      for_current_g(0),
      for_current_real_g(0),
      for_current_eval_context(for_current_state, 0, true, &statistics),
      bac_current_state(regression_state_registry.create_goal_state(
          partial_state_task_proxy.create_state(
              move(regression_task->get_goal_state_values())))),
      bac_current_successor_id(StateID::no_state),
      bac_current_operator_id(OperatorID::no_operator),
      bac_current_g(0),
      bac_current_real_g(0),
      bac_current_eval_context(bac_current_state, 0, true, &statistics) {
  /*
    We initialize current_eval_context in such a way that the initial node
    counts as "preferred".
  */
}

void BidirectionalLazySearch::for_set_preferred_operator_evaluators(
    vector<shared_ptr<Evaluator>> &evaluators) {
  for_preferred_operator_evaluators = evaluators;
}

void BidirectionalLazySearch::bac_set_preferred_operator_evaluators(
    vector<shared_ptr<Evaluator>> &evaluators) {
  bac_preferred_operator_evaluators = evaluators;
}

void BidirectionalLazySearch::initialize() {
  cout << "Conducting lazy best first search, (real) bound = " << bound << endl;

  assert(for_open_list);
  assert(bac_open_list);
  set<Evaluator *> for_evals;
  set<Evaluator *> bac_evals;
  for_open_list->get_path_dependent_evaluators(for_evals);
  bac_open_list->get_path_dependent_evaluators(bac_evals);

  // Add evaluators that are used for preferred operators (in case they are
  // not also used in the open list).
  for (const shared_ptr<Evaluator> &evaluator :
       for_preferred_operator_evaluators) {
    evaluator->get_path_dependent_evaluators(for_evals);
  }

  for (const shared_ptr<Evaluator> &evaluator :
       bac_preferred_operator_evaluators) {
    evaluator->get_path_dependent_evaluators(bac_evals);
  }

  for_path_dependent_evaluators.assign(for_evals.begin(), for_evals.end());
  bac_path_dependent_evaluators.assign(bac_evals.begin(), bac_evals.end());
  const GlobalState &initial_state = state_registry.get_initial_state();
  for (Evaluator *evaluator : for_path_dependent_evaluators) {
    evaluator->notify_initial_state(initial_state);
  }
  for (Evaluator *evaluator : bac_path_dependent_evaluators) {
    evaluator->notify_initial_state(initial_state);
  }

  directions[for_current_state] = FORWARD;
  directions[bac_current_state] = BACKWARD;
}

vector<OperatorID> BidirectionalLazySearch::get_successor_operators(
    const ordered_set::OrderedSet<OperatorID> &preferred_operators) const {
  vector<OperatorID> applicable_operators;
  successor_generator.generate_applicable_ops(for_current_state,
                                              applicable_operators);

  if (randomize_successors) {
    rng->shuffle(applicable_operators);
  }

  if (preferred_successors_first) {
    ordered_set::OrderedSet<OperatorID> successor_operators;
    for (OperatorID op_id : preferred_operators) {
      successor_operators.insert(op_id);
    }
    for (OperatorID op_id : applicable_operators) {
      successor_operators.insert(op_id);
    }
    return successor_operators.pop_as_vector();
  } else {
    return applicable_operators;
  }
}

vector<OperatorID> BidirectionalLazySearch::get_predecessor_operators(
    const ordered_set::OrderedSet<OperatorID> &preferred_operators) const {
  vector<OperatorID> applicable_operators;
  regression_successor_generator.generate_applicable_ops(bac_current_state,
                                                         applicable_operators);

  if (randomize_successors) {
    rng->shuffle(applicable_operators);
  }

  if (preferred_successors_first) {
    ordered_set::OrderedSet<OperatorID> successor_operators;
    for (OperatorID op_id : preferred_operators) {
      successor_operators.insert(op_id);
    }
    for (OperatorID op_id : applicable_operators) {
      successor_operators.insert(op_id);
    }
    return successor_operators.pop_as_vector();
  } else {
    return applicable_operators;
  }
}

void BidirectionalLazySearch::generate_successors() {
  ordered_set::OrderedSet<OperatorID> preferred_operators;
  for (const shared_ptr<Evaluator> &preferred_operator_evaluator :
       for_preferred_operator_evaluators) {
    collect_preferred_operators(for_current_eval_context,
                                preferred_operator_evaluator.get(),
                                preferred_operators);
  }
  if (randomize_successors) {
    preferred_operators.shuffle(*rng);
  }

  vector<OperatorID> successor_operators =
      get_successor_operators(preferred_operators);

  statistics.inc_generated(successor_operators.size());

  for (OperatorID op_id : successor_operators) {
    OperatorProxy op = task_proxy.get_operators()[op_id];
    int new_g = for_current_g + get_adjusted_cost(op);
    int new_real_g = for_current_real_g + op.get_cost();
    bool is_preferred = preferred_operators.contains(op_id);
    if (new_real_g < bound) {
      EvaluationContext new_eval_context(for_current_eval_context.get_cache(),
                                         new_g, is_preferred, nullptr);
      for_open_list->insert(new_eval_context,
                            make_pair(for_current_state.get_id(), op_id));
    }
  }
}

void BidirectionalLazySearch::generate_predecessors() {
  ordered_set::OrderedSet<OperatorID> preferred_operators;
  for (const shared_ptr<Evaluator> &preferred_operator_evaluator :
       bac_preferred_operator_evaluators) {
    collect_preferred_operators(bac_current_eval_context,
                                preferred_operator_evaluator.get(),
                                preferred_operators);
  }
  if (randomize_successors) {
    preferred_operators.shuffle(*rng);
  }

  vector<OperatorID> successor_operators =
      get_predecessor_operators(preferred_operators);

  statistics.inc_generated(successor_operators.size());

  for (OperatorID op_id : successor_operators) {
    OperatorProxy op = regression_task_proxy.get_operators()[op_id];
    int new_g = bac_current_g + get_adjusted_cost(op);
    int new_real_g = bac_current_real_g + op.get_cost();
    bool is_preferred = preferred_operators.contains(op_id);
    if (new_real_g < bound) {
      EvaluationContext new_eval_context(bac_current_eval_context.get_cache(),
                                         new_g, is_preferred, nullptr);
      bac_open_list->insert(new_eval_context,
                            make_pair(bac_current_state.get_id(), op_id));
    }
  }
}

SearchStatus BidirectionalLazySearch::for_fetch_next_state() {
  if (for_open_list->empty()) {
    cout << "Completely explored state space -- no solution!" << endl;
    return FAILED;
  }
  EdgeOpenListEntry next = for_open_list->remove_min();

  for_current_predecessor_id = next.first;
  for_current_operator_id = next.second;
  GlobalState current_predecessor =
      regression_state_registry.lookup_state(for_current_predecessor_id);
  OperatorProxy current_operator =
      task_proxy.get_operators()[for_current_operator_id];
  assert(task_properties::is_applicable(current_operator,
                                        current_predecessor.unpack()));
  for_current_state = regression_state_registry.get_successor_state(
      current_predecessor, current_operator);

  SearchNode pred_node =
      partial_state_search_space.get_node(current_predecessor);
  for_current_g = pred_node.get_g() + get_adjusted_cost(current_operator);
  for_current_real_g = pred_node.get_real_g() + current_operator.get_cost();

  if (directions[for_current_state] == BACKWARD) {
    meet_set_plan(FORWARD, current_predecessor, for_current_operator_id,
                  for_current_state);
    return SOLVED;
  } else {
    directions[for_current_state] = FORWARD;
  }

  /*
    Note: We mark the node in current_eval_context as "preferred"
    here. This probably doesn't matter much either way because the
    node has already been selected for expansion, but eventually we
    should think more deeply about which path information to
    associate with the expanded vs. evaluated nodes in lazy search
    and where to obtain it from.
  */
  for_current_eval_context =
      EvaluationContext(for_current_state, for_current_g, true, &statistics);

  current_direction = FORWARD;

  return IN_PROGRESS;
}

SearchStatus BidirectionalLazySearch::bac_fetch_next_state() {
  StateID state_id = StateID::no_state;

  while (state_id == StateID::no_state) {
    if (bac_open_list->empty()) {
      cout << "Completely explored state space -- no solution!" << endl;
      return FAILED;
    }

    EdgeOpenListEntry next = bac_open_list->remove_min();

    bac_current_successor_id = next.first;
    bac_current_operator_id = next.second;
    GlobalState current_successor =
        regression_state_registry.lookup_state(bac_current_successor_id);
    OperatorProxy current_operator =
        regression_task_proxy.get_operators()[bac_current_operator_id];
    assert(task_properties::is_applicable(current_operator,
                                          current_successor.unpack()));
    state_id = regression_state_registry.get_predecessor_state(
        current_successor, current_operator);

    if (state_id == StateID::no_state) continue;

    if (prune_goal) {
      GlobalState pre_state = regression_state_registry.lookup_state(state_id);
      if (task_properties::is_goal_state(task_proxy, pre_state)) continue;
    }

    bac_current_state = regression_state_registry.lookup_state(state_id);
    SearchNode pred_node =
        partial_state_search_space.get_node(current_successor);
    bac_current_g = pred_node.get_g() + get_adjusted_cost(current_operator);
    bac_current_real_g = pred_node.get_real_g() + current_operator.get_cost();

    if (directions[bac_current_state] == FORWARD) {
      meet_set_plan(BACKWARD, bac_current_state, bac_current_operator_id,
                    current_successor);
      return SOLVED;
    } else {
      directions[bac_current_state] = BACKWARD;
    }

    /*
      Note: We mark the node in current_eval_context as "preferred"
      here. This probably doesn't matter much either way because the
      node has already been selected bac expansion, but eventually we
      should think more deeply about which path inbacmation to
      associate with the expanded vs. evaluated nodes in lazy search
      and where to obtain it from.
    */
  }

  bac_open_list->set_goal(bac_current_state);
  bac_current_eval_context =
      EvaluationContext(regression_state_registry.get_initial_state(),
                        bac_current_g, true, &statistics);

  current_direction = BACKWARD;

  return IN_PROGRESS;
}

SearchStatus BidirectionalLazySearch::step() {
  if (current_direction == FORWARD) return for_step();

  return bac_step();
}

SearchStatus BidirectionalLazySearch::for_step() {
  // Invariants:
  // - current_state is the next state for which we want to compute the
  // heuristic.
  // - current_predecessor is a permanent pointer to the predecessor of that
  // state.
  // - current_operator is the operator which leads to current_state from
  // predecessor.
  // - current_g is the g value of the current state according to the
  // cost_type
  // - current_real_g is the g value of the current state (using real costs)

  SearchNode node = partial_state_search_space.get_node(for_current_state);
  bool reopen = reopen_closed_nodes && !node.is_new() && !node.is_dead_end() &&
                (for_current_g < node.get_g());

  if (node.is_new() || reopen) {
    if (for_current_operator_id != OperatorID::no_operator) {
      assert(for_current_predecessor_id != StateID::no_state);
      GlobalState parent_state =
          regression_state_registry.lookup_state(for_current_predecessor_id);
      for (Evaluator *evaluator : for_path_dependent_evaluators)
        evaluator->notify_state_transition(
            parent_state, for_current_operator_id, for_current_state);
    }
    statistics.inc_evaluated_states();
    if (!for_open_list->is_dead_end(for_current_eval_context)) {
      // TODO: Generalize code for using multiple evaluators.
      if (for_current_predecessor_id == StateID::no_state) {
        node.open_initial();
        if (search_progress.check_progress(for_current_eval_context))
          statistics.print_checkpoint_line(for_current_g);
      } else {
        GlobalState parent_state =
            regression_state_registry.lookup_state(for_current_predecessor_id);
        SearchNode parent_node =
            partial_state_search_space.get_node(parent_state);
        OperatorProxy current_operator =
            task_proxy.get_operators()[for_current_operator_id];
        if (reopen) {
          node.reopen(parent_node, current_operator,
                      get_adjusted_cost(current_operator));
          statistics.inc_reopened();
        } else {
          node.open(parent_node, current_operator,
                    get_adjusted_cost(current_operator));
        }
      }
      node.close();
      if (check_goal_and_set_plan(for_current_state)) return SOLVED;
      if (search_progress.check_progress(for_current_eval_context)) {
        statistics.print_checkpoint_line(for_current_g);
        for_reward_progress();
      }
      generate_successors();
      statistics.inc_expanded();
    } else {
      node.mark_as_dead_end();
      statistics.inc_dead_ends();
    }
    if (for_current_predecessor_id == StateID::no_state) {
      print_initial_evaluator_values(for_current_eval_context);
    }
  }

  if (for_current_predecessor_id == StateID::no_state) {
    current_direction = BACKWARD;
    return IN_PROGRESS;
  }

  return bac_fetch_next_state();
}

SearchStatus BidirectionalLazySearch::bac_step() {
  // Invariants:
  // - current_state is the next state for which we want to compute the
  // heuristic.
  // - current_predecessor is a permanent pointer to the predecessor of that
  // state.
  // - current_operator is the operator which leads to current_state from
  // predecessor.
  // - current_g is the g value of the current state according to the
  // cost_type
  // - current_real_g is the g value of the current state (using real costs)

  SearchNode node = partial_state_search_space.get_node(bac_current_state);
  bool reopen = reopen_closed_nodes && !node.is_new() && !node.is_dead_end() &&
                (for_current_g < node.get_g());

  if (node.is_new() || reopen) {
    if (bac_current_operator_id != OperatorID::no_operator) {
      assert(bac_current_successor_id != StateID::no_state);
      GlobalState child_state =
          regression_state_registry.lookup_state(bac_current_successor_id);
      for (Evaluator *evaluator : bac_path_dependent_evaluators)
        evaluator->notify_state_transition(
            bac_current_state, bac_current_operator_id, child_state);
    }
    statistics.inc_evaluated_states();
    if (!bac_open_list->is_dead_end(bac_current_eval_context)) {
      // TODO: Generalize code for using multiple evaluators.
      if (bac_current_successor_id == StateID::no_state) {
        node.open_initial();
        if (search_progress.check_progress(bac_current_eval_context))
          statistics.print_checkpoint_line(bac_current_g);
      } else {
        GlobalState child_state =
            regression_state_registry.lookup_state(bac_current_successor_id);
        SearchNode child_node =
            partial_state_search_space.get_node(child_state);
        OperatorProxy current_operator =
            regression_task_proxy.get_operators()[bac_current_operator_id];
        if (reopen) {
          node.reopen(child_node, current_operator,
                      get_adjusted_cost(current_operator));
          statistics.inc_reopened();
        } else {
          node.open(child_node, current_operator,
                    get_adjusted_cost(current_operator));
        }
      }
      node.close();
      if (check_initial_and_set_plan(bac_current_state)) return SOLVED;
      if (search_progress.check_progress(bac_current_eval_context)) {
        statistics.print_checkpoint_line(bac_current_g);
        bac_reward_progress();
      }
      generate_predecessors();
      statistics.inc_expanded();
    } else {
      node.mark_as_dead_end();
      statistics.inc_dead_ends();
    }
    if (bac_current_successor_id == StateID::no_state) {
      print_initial_evaluator_values(bac_current_eval_context);
    }
  }
  return for_fetch_next_state();
}

void BidirectionalLazySearch::for_reward_progress() {
  for_open_list->boost_preferred();
}

void BidirectionalLazySearch::bac_reward_progress() {
  bac_open_list->boost_preferred();
}

void BidirectionalLazySearch::print_statistics() const {
  statistics.print_detailed_statistics();
  partial_state_search_space.print_statistics();
}

bool BidirectionalLazySearch::check_goal_and_set_plan(
    const GlobalState &state) {
  if (task_properties::is_goal_state(task_proxy, state)) {
    Plan plan;
    partial_state_search_space.trace_path(state, plan);
    set_plan(plan);
    return true;
  }

  return false;
}

bool BidirectionalLazySearch::check_initial_and_set_plan(
    const GlobalState &state) {
  const GlobalState &initial_state =
      regression_state_registry.get_initial_state();
  VariablesProxy variables = partial_state_task_proxy.get_variables();

  for (auto var : variables) {
    if (state[var.get_id()] != var.get_domain_size() - 1 &&
        state[var.get_id()] != initial_state[var.get_id()]) {
      return false;
    }
  }

  Plan plan;
  partial_state_search_space.trace_path(state, plan);
  reverse(plan.begin(), plan.end());
  set_plan(plan);

  return true;
}

bool BidirectionalLazySearch::check_meeting_and_set_plan(
    const GlobalState &s_f, const GlobalState &s_b) {
  VariablesProxy variables = partial_state_task_proxy.get_variables();

  for (auto var : variables) {
    if (s_b[var.get_id()] != var.get_domain_size() - 1 &&
        s_f[var.get_id()] != s_b[var.get_id()]) {
      return false;
    }
  }

  Plan plan;
  partial_state_search_space.trace_path(s_f, plan);
  cout << "#forward actions: " << plan.size() << endl;
  Plan regression_plan;
  partial_state_search_space.trace_path(s_b, regression_plan);
  reverse(regression_plan.begin(), regression_plan.end());
  cout << "#backward actions: " << regression_plan.size() << endl;
  plan.insert(plan.end(), regression_plan.begin(), regression_plan.end());
  set_plan(plan);

  return true;
}

void BidirectionalLazySearch::meet_set_plan(Direction d, const GlobalState &s_f,
                                            OperatorID op_id,
                                            const GlobalState &s_b) {
  Plan plan;
  partial_state_search_space.trace_path(s_f, plan);

  if (d == Direction::FORWARD) plan.push_back(op_id);

  cout << "#forward actions: " << plan.size() << endl;
  Plan regression_plan;
  partial_state_search_space.trace_path(s_b, regression_plan);

  if (d == Direction::BACKWARD) regression_plan.push_back(op_id);

  reverse(regression_plan.begin(), regression_plan.end());
  cout << "#backward actions: " << regression_plan.size() << endl;
  plan.insert(plan.end(), regression_plan.begin(), regression_plan.end());
  set_plan(plan);
}

void add_options_to_parser(OptionParser &parser) {
  SearchEngine::add_pruning_option(parser);
  SearchEngine::add_options_to_parser(parser);
}
}  // namespace bidirectional_lazy_search
