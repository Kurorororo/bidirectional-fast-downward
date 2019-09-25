#include "regression_front_to_front_eager_search.h"

#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../front_to_front/front_to_front_open_list_factory.h"
#include "../option_parser.h"

#include "../algorithms/ordered_set.h"
#include "../task_utils/successor_generator.h"

#include "../utils/logging.h"

#include <cassert>
#include <cstdlib>
#include <memory>
#include <optional.hh>
#include <set>

using namespace std;

namespace regression_front_to_front_eager_search {
RegressionFrontToFrontEagerSearch::RegressionFrontToFrontEagerSearch(
    const Options &opts)
    : SearchEngine(opts),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      partial_state_task(tasks::PartialStateTask::get_partial_state_task()),
      partial_state_task_proxy(*partial_state_task),
      regression_state_registry(partial_state_task_proxy),
      partial_state_search_space(regression_state_registry),
      regression_task(tasks::RegressionTask::get_regression_task()),
      regression_task_proxy(*regression_task),
      regression_successor_generator(regression_task),
      current_direction(Direction::FORWARD) {
  open_lists[Direction::FORWARD] =
      opts.get<shared_ptr<FrontToFrontOpenListFactory>>("open_f")
          ->create_state_open_list();
  open_lists[Direction::BACKWARD] =
      opts.get<shared_ptr<FrontToFrontOpenListFactory>>("open_b")
          ->create_state_open_list();
  f_evaluators[Direction::FORWARD] =
      opts.get<shared_ptr<Evaluator>>("f_eval_f", nullptr);
  f_evaluators[Direction::BACKWARD] =
      opts.get<shared_ptr<Evaluator>>("f_eval_b", nullptr);
  preferred_operator_evaluators[Direction::FORWARD] =
      opts.get_list<shared_ptr<FrontToFrontHeuristic>>("preferred_f");
  preferred_operator_evaluators[Direction::BACKWARD] =
      opts.get_list<shared_ptr<FrontToFrontHeuristic>>("preferred_b");
}

void RegressionFrontToFrontEagerSearch::initialize() {
  cout << "Conducting front to front best first search"
       << (reopen_closed_nodes ? " with" : " without")
       << " reopening closed nodes, (real) bound = " << bound << endl;
  assert(open_lists[Direction::FORWARD]);
  assert(open_lists[Direction::BACKWARD]);

  vector<Direction> ds{Direction::FORWARD, Direction::BACKWARD};

  for (auto d : ds) {
    set<Evaluator *> evals;
    open_lists[d]->get_path_dependent_evaluators(evals);

    /*
      Collect path-dependent evaluators that are used for preferred operators
      (in case they are not also used in the open list).
    */
    for (const shared_ptr<Evaluator> &evaluator :
         preferred_operator_evaluators[d]) {
      evaluator->get_path_dependent_evaluators(evals);
    }

    /*
      Collect path-dependent evaluators that are used in the f_evaluator.
      They are usually also used in the open list and will hence already be
      included, but we want to be sure.
    */
    if (f_evaluators[d]) {
      f_evaluators[d]->get_path_dependent_evaluators(evals);
    }

    /*
      Collect path-dependent evaluators that are used in the lazy_evaluator
      (in case they are not already included).
    */

    path_dependent_evaluators[d].assign(evals.begin(), evals.end());
  }

  const GlobalState &initial_state =
      regression_state_registry.get_initial_state();
  for (Evaluator *evaluator : path_dependent_evaluators[Direction::FORWARD]) {
    evaluator->notify_initial_state(initial_state);
  }
  directions[initial_state] = Direction::FORWARD;

  vector<int> goal_state_values = regression_task->get_goal_state_values();
  State goal_state =
      partial_state_task_proxy.create_state(move(goal_state_values));
  const GlobalState global_goal_state =
      regression_state_registry.create_goal_state(goal_state);
  for (Evaluator *evaluator : path_dependent_evaluators[Direction::BACKWARD]) {
    evaluator->notify_initial_state(initial_state);
  }
  directions[global_goal_state] = Direction::BACKWARD;

  /*
    Note: we consider the initial state as reached by a preferred
    operator.
  */
  open_lists[Direction::FORWARD]->set_goal(global_goal_state);
  EvaluationContext eval_context(initial_state, 0, true, &statistics);

  statistics.inc_evaluated_states();

  if (open_lists[Direction::FORWARD]->is_dead_end(eval_context)) {
    cout << "Initial state is a dead end." << endl;
  } else {
    if (search_progress.check_progress(eval_context)) {
      statistics.print_checkpoint_line(0);
    }
    start_f_value_statistics(Direction::FORWARD, eval_context);
    SearchNode node_f = partial_state_search_space.get_node(initial_state);
    node_f.open_initial();

    open_lists[Direction::FORWARD]->insert(eval_context,
                                           initial_state.get_id());
  }

  SearchNode node_b = partial_state_search_space.get_node(global_goal_state);
  node_b.open_initial();

  open_lists[Direction::BACKWARD]->insert(eval_context,
                                          global_goal_state.get_id());

  print_initial_evaluator_values(eval_context);
}

void RegressionFrontToFrontEagerSearch::print_statistics() const {
  statistics.print_detailed_statistics();
  partial_state_search_space.print_statistics();
}

SearchStatus RegressionFrontToFrontEagerSearch::step() {
  tl::optional<SearchNode> node;
  while (true) {
    if (open_lists[Direction::FORWARD]->empty() &&
        open_lists[Direction::BACKWARD]->empty()) {
      cout << "Completely explored state space -- no solution!" << endl;
      return FAILED;
    }

    if (open_lists[Direction::FORWARD]->empty())
      current_direction = Direction::BACKWARD;
    if (open_lists[Direction::BACKWARD]->empty())
      current_direction = Direction::FORWARD;

    StateID id = open_lists[current_direction]->remove_min();
    // TODO is there a way we can avoid creating the state here and then
    //      recreate it outside of this function with node.get_state()?
    //      One way would be to store GlobalState objects inside SearchNodes
    //      instead of StateIDs
    GlobalState s = regression_state_registry.lookup_state(id);
    node.emplace(partial_state_search_space.get_node(s));

    if (node->is_closed()) continue;

    /*
      We can pass calculate_preferred=false here since preferred
      operators are computed when the state is expanded.
    */
    EvaluationContext eval_context(s, node->get_g(), false, &statistics);

    node->close();
    assert(!node->is_dead_end());
    update_f_value_statistics(current_direction, eval_context);
    statistics.inc_expanded();
    break;
  }

  SearchStatus status = IN_PROGRESS;

  if (current_direction == Direction::FORWARD) {
    status = forward_step(node);
  } else {
    status = backward_step(node);
  }

  current_direction = current_direction == Direction::FORWARD
                          ? Direction::BACKWARD
                          : Direction::FORWARD;

  return status;
}

void RegressionFrontToFrontEagerSearch::reward_progress(Direction d) {
  // Boost the "preferred operator" open lists somewhat whenever
  // one of the heuristics finds a state with a new best h value.

  open_lists[d]->boost_preferred();
}

void RegressionFrontToFrontEagerSearch::dump_search_space() const {
  partial_state_search_space.dump(partial_state_task_proxy);
}

void RegressionFrontToFrontEagerSearch::start_f_value_statistics(
    Direction d, EvaluationContext &eval_context) {
  if (f_evaluators[d]) {
    int f_value = eval_context.get_evaluator_value(f_evaluators[d].get());
    statistics.report_f_value_progress(f_value);
  }
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every
   state. */
void RegressionFrontToFrontEagerSearch::update_f_value_statistics(
    Direction d, EvaluationContext &eval_context) {
  if (f_evaluators[d]) {
    int f_value = eval_context.get_evaluator_value(f_evaluators[d].get());
    statistics.report_f_value_progress(f_value);
  }
}

bool RegressionFrontToFrontEagerSearch::check_initial_and_set_plan(
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

SearchStatus RegressionFrontToFrontEagerSearch::forward_step(
    const tl::optional<SearchNode> &node) {
  GlobalState state = node->get_state();

  if (check_goal_and_set_plan(state)) {
    cout << "#forward actions: " << get_plan().size() << endl;
    cout << "#backward actions: " << 0 << endl;
    return SOLVED;
  }

  vector<OperatorID> applicable_ops;
  successor_generator.generate_applicable_ops(state, applicable_ops);

  ordered_set::OrderedSet<OperatorID> preferred_operators;
  EvaluationContext eval_context(state, node->get_g(), false, &statistics,
                                 true);

  for (const shared_ptr<Evaluator> &preferred_operator_evaluator :
       preferred_operator_evaluators[Direction::FORWARD]) {
    collect_preferred_operators(
        eval_context, preferred_operator_evaluator.get(), preferred_operators);
  }

  if (!open_lists[Direction::BACKWARD]->empty()) {
    auto other_top = open_lists[Direction::BACKWARD]->get_min_value_and_entry();
    GlobalState frontier_state =
        regression_state_registry.lookup_state(other_top.second);
    open_lists[Direction::FORWARD]->set_goal(frontier_state);
  }

  for (OperatorID op_id : applicable_ops) {
    OperatorProxy op = task_proxy.get_operators()[op_id];
    if ((node->get_real_g() + op.get_cost()) >= bound) continue;

    GlobalState succ_state =
        regression_state_registry.get_successor_state(state, op);
    statistics.inc_generated();
    bool is_preferred = preferred_operators.contains(op_id);

    SearchNode succ_node = partial_state_search_space.get_node(succ_state);

    for (Evaluator *evaluator : path_dependent_evaluators[Direction::FORWARD]) {
      evaluator->notify_state_transition(state, op_id, succ_state);
    }

    if (check_meeting_and_set_plan(Direction::FORWARD, state, op_id,
                                   succ_state))
      return SOLVED;

    if (succ_node.is_dead_end()) continue;

    if (succ_node.is_new()) {
      int succ_g = node->get_g() + get_adjusted_cost(op);

      EvaluationContext succ_eval_context(succ_state, succ_g, is_preferred,
                                          &statistics);
      statistics.inc_evaluated_states();

      if (open_lists[Direction::FORWARD]->is_dead_end(succ_eval_context)) {
        succ_node.mark_as_dead_end();
        statistics.inc_dead_ends();
        continue;
      }
      succ_node.open(*node, op, get_adjusted_cost(op));
      directions[succ_state] = Direction::FORWARD;

      open_lists[Direction::FORWARD]->insert(succ_eval_context,
                                             succ_state.get_id());
      if (search_progress.check_progress(succ_eval_context)) {
        statistics.print_checkpoint_line(succ_node.get_g());
        reward_progress(current_direction);
      }
    } else if (succ_node.get_g() > node->get_g() + get_adjusted_cost(op)) {
      if (reopen_closed_nodes) {
        if (succ_node.is_closed()) {
          statistics.inc_reopened();
        }
        succ_node.reopen(*node, op, get_adjusted_cost(op));

        EvaluationContext succ_eval_context(succ_state, succ_node.get_g(),
                                            is_preferred, &statistics);
        open_lists[Direction::FORWARD]->insert(succ_eval_context,
                                               succ_state.get_id());
      } else {
        succ_node.update_parent(*node, op, get_adjusted_cost(op));
      }
    }
  }

  return IN_PROGRESS;
}

SearchStatus RegressionFrontToFrontEagerSearch::backward_step(
    const tl::optional<SearchNode> &node) {
  GlobalState state = node->get_state();

  if (check_initial_and_set_plan(state)) {
    cout << "#forward actions: " << 0 << endl;
    cout << "#backward actions: " << get_plan().size() << endl;
    return SOLVED;
  }

  vector<OperatorID> applicable_ops;
  regression_successor_generator.generate_applicable_ops(state, applicable_ops);

  ordered_set::OrderedSet<OperatorID> preferred_operators;

  GlobalState frontier_state = regression_state_registry.get_initial_state();
  if (!open_lists[Direction::FORWARD]->empty()) {
    auto other_top = open_lists[Direction::FORWARD]->get_min_value_and_entry();
    frontier_state = regression_state_registry.lookup_state(other_top.second);
  }

  open_lists[Direction::BACKWARD]->set_goal(state);
  EvaluationContext eval_context(frontier_state, node->get_g(), false,
                                 &statistics, true);

  for (const shared_ptr<Evaluator> &preferred_operator_evaluator :
       preferred_operator_evaluators[Direction::BACKWARD]) {
    collect_preferred_operators(
        eval_context, preferred_operator_evaluator.get(), preferred_operators);
  }

  for (OperatorID op_id : applicable_ops) {
    OperatorProxy op = regression_task_proxy.get_operators()[op_id];
    if ((node->get_real_g() + op.get_cost()) >= bound) continue;

    StateID pre_state_id =
        regression_state_registry.get_predecessor_state(state, op);

    if (pre_state_id == StateID::no_state) continue;

    GlobalState pre_state =
        regression_state_registry.lookup_state(pre_state_id);
    statistics.inc_generated();
    bool is_preferred = preferred_operators.contains(op_id);

    SearchNode pre_node = partial_state_search_space.get_node(pre_state);

    for (Evaluator *evaluator :
         path_dependent_evaluators[Direction::BACKWARD]) {
      evaluator->notify_state_transition(pre_state, op_id, state);
    }

    if (check_meeting_and_set_plan(Direction::BACKWARD, state, op_id,
                                   pre_state))
      return SOLVED;

    if (pre_node.is_dead_end()) continue;

    if (pre_node.is_new()) {
      int succ_g = node->get_g() + get_adjusted_cost(op);

      open_lists[Direction::BACKWARD]->set_goal(pre_state);
      EvaluationContext pre_eval_context(frontier_state, succ_g, is_preferred,
                                         &statistics);
      statistics.inc_evaluated_states();

      if (open_lists[Direction::BACKWARD]->is_dead_end(pre_eval_context)) {
        pre_node.mark_as_dead_end();
        statistics.inc_dead_ends();
        continue;
      }
      pre_node.open(*node, op, get_adjusted_cost(op));
      directions[pre_state] = Direction::BACKWARD;

      open_lists[Direction::BACKWARD]->insert(pre_eval_context,
                                              pre_state.get_id());
      if (search_progress.check_progress(pre_eval_context)) {
        statistics.print_checkpoint_line(pre_node.get_g());
        reward_progress(current_direction);
      }
    } else if (pre_node.get_g() > node->get_g() + get_adjusted_cost(op)) {
      if (reopen_closed_nodes) {
        if (pre_node.is_closed()) {
          statistics.inc_reopened();
        }
        pre_node.reopen(*node, op, get_adjusted_cost(op));

        open_lists[Direction::BACKWARD]->set_goal(pre_state);
        EvaluationContext pre_eval_context(frontier_state, pre_node.get_g(),
                                           is_preferred, &statistics);
        open_lists[Direction::BACKWARD]->insert(pre_eval_context,
                                                pre_state.get_id());
      } else {
        pre_node.update_parent(*node, op, get_adjusted_cost(op));
      }
    }
  }

  return IN_PROGRESS;
}

bool RegressionFrontToFrontEagerSearch::check_meeting_and_set_plan(
    Direction d, const GlobalState &parent, OperatorID op_id,
    const GlobalState &state) {
  SearchNode node = partial_state_search_space.get_node(state);

  if (!node.is_new() && directions[state] != Direction::NONE &&
      directions[state] != d) {
    cout << "Solution found!" << endl;

    Plan plan1;
    partial_state_search_space.trace_path(state, plan1);
    Plan plan2;
    partial_state_search_space.trace_path(parent, plan2);

    if (d == Direction::FORWARD) {
      cout << "#forward actions: " << plan2.size() + 1 << endl;
      cout << "#backward actions: " << plan1.size() << endl;
      reverse(plan1.begin(), plan1.end());
      plan2.push_back(op_id);
      plan2.insert(plan2.end(), plan1.begin(), plan1.end());
      set_plan(plan2);
    } else {
      cout << "#forward actions: " << plan1.size() << endl;
      cout << "#backward actions: " << plan2.size() + 1 << endl;
      reverse(plan2.begin(), plan2.end());
      plan1.push_back(op_id);
      plan1.insert(plan1.end(), plan2.begin(), plan2.end());
      set_plan(plan1);
    }

    return true;
  }

  return false;
}

void add_options_to_parser(OptionParser &parser) {
  SearchEngine::add_pruning_option(parser);
  SearchEngine::add_options_to_parser(parser);
}

}  // namespace regression_front_to_front_eager_search
