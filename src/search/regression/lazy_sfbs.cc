#include "lazy_sfbs.h"

#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../open_list_factory.h"
#include "../option_parser.h"
#include "../pruning_method.h"

#include "../algorithms/ordered_set.h"
#include "../front_to_front/front_to_front_open_list_factory.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"
#include "regression_successor_generator.h"

#include <cassert>
#include <cstdlib>
#include <memory>
#include <optional.hh>
#include <set>
#include "../utils/logging.h"

using namespace std;

namespace lazy_sfbs {
LazySFBS::LazySFBS(const Options &opts)
    : SearchEngine(opts),
      reopen_closed_nodes(opts.get<bool>("reopen_closed", false)),
      is_initial(true),
      current_direction(FORWARD),
      open_list(opts.get<shared_ptr<FrontToFrontOpenListFactory>>("open")
                    ->create_frontier_open_list()),
      f_evaluator(opts.get<shared_ptr<Evaluator>>("f_eval", nullptr)),
      preferred_operator_evaluators(
          opts.get_list<shared_ptr<FrontToFrontHeuristic>>("preferred")),
      directions(NONE),
      state_operator_id(OperatorID::no_operator),
      partial_state_task(tasks::PartialStateTask::get_partial_state_task()),
      partial_state_task_proxy(*partial_state_task),
      regression_state_registry(partial_state_task_proxy),
      partial_state_search_space(regression_state_registry),
      regression_task(tasks::RegressionTask::get_regression_task()),
      regression_task_proxy(*regression_task),
      regression_successor_generator(regression_task),
      for_current_state(regression_state_registry.get_initial_state()),
      bac_current_state(regression_state_registry.create_goal_state(
          partial_state_task_proxy.create_state(
              move(regression_task->get_goal_state_values())))),
      current_g(0),
      current_eval_context(for_current_state, 0, true, &statistics) {}

void LazySFBS::initialize() {
  cout << "Conducting best first search"
       << (reopen_closed_nodes ? " with" : " without")
       << " reopening closed nodes, (real) bound = " << bound << endl;
  assert(open_list);

  set<Evaluator *> evals;
  open_list->get_path_dependent_evaluators(evals);

  /*
    Collect path-dependent evaluators that are used for preferred operators
    (in case they are not also used in the open list).
  */
  /*
    Collect path-dependent evaluators that are used in the f_evaluator.
    They are usually also used in the open list and will hence already be
    included, but we want to be sure.
  */
  if (f_evaluator) {
    f_evaluator->get_path_dependent_evaluators(evals);
  }

  for (const shared_ptr<Evaluator> &evaluator : preferred_operator_evaluators) {
    evaluator->get_path_dependent_evaluators(evals);
  }

  path_dependent_evaluators.assign(evals.begin(), evals.end());

  for (Evaluator *evaluator : path_dependent_evaluators) {
    evaluator->notify_initial_state(for_current_state);
  }
  for (Evaluator *evaluator : path_dependent_evaluators) {
    evaluator->notify_initial_state(for_current_state);
  }

  SearchNode for_node = partial_state_search_space.get_node(for_current_state);
  for_node.open_initial();
  SearchNode bac_node = partial_state_search_space.get_node(bac_current_state);
  bac_node.open_initial();

  directions[for_current_state] = FORWARD;
  directions[bac_current_state] = BACKWARD;
  n_steps[for_current_state] = 0;
  n_steps[bac_current_state] = 0;
}

SearchStatus LazySFBS::step() {
  if (current_direction == FORWARD) return for_step();

  return bac_step();
}

SearchStatus LazySFBS::fetch_next_state() {
  while (true) {
    if (open_list->empty()) {
      cout << "Completely explored state space -- no solution!" << endl;
      return FAILED;
    }
    FrontierOpenListEntry next = open_list->remove_min();

    StateID for_id = next.first;
    for_current_state = regression_state_registry.lookup_state(for_id);
    StateID bac_id = next.second;
    bac_current_state = regression_state_registry.lookup_state(bac_id);

    if (check_goal_and_set_plan(for_current_state)) return SOLVED;

    if (check_initial_and_set_plan(bac_current_state)) return SOLVED;

    if (check_meeting_and_set_plan(for_current_state, bac_current_state))
      return SOLVED;

    auto id_pair = make_pair(for_id.get_value(), bac_id.get_value());

    if (closed_list.find(id_pair) != closed_list.end()) continue;

    closed_list.insert(id_pair);

    open_list->set_goal(bac_current_state);

    if (n_steps[for_current_state] <= n_steps[bac_current_state]) {
      SearchNode node = partial_state_search_space.get_node(for_current_state);
      StateID parent_id = node.get_parent_state_id();

      if (parent_id != StateID::no_state) {
        GlobalState parent_state =
            regression_state_registry.lookup_state(parent_id);
        OperatorID op_id = state_operator_id[for_current_state];

        for (Evaluator *evaluator : path_dependent_evaluators) {
          evaluator->notify_state_transition(parent_state, op_id,
                                             for_current_state);
        }
      }

      current_g = node.get_g();
      current_direction = FORWARD;
    } else {
      SearchNode node = partial_state_search_space.get_node(bac_current_state);
      StateID parent_id = node.get_parent_state_id();

      if (parent_id != StateID::no_state) {
        GlobalState parent_state =
            regression_state_registry.lookup_state(parent_id);
        OperatorID op_id = state_operator_id[bac_current_state];

        for (Evaluator *evaluator : path_dependent_evaluators) {
          evaluator->notify_state_transition(bac_current_state, op_id,
                                             parent_state);
        }
      }

      current_g = node.get_g();
      current_direction = BACKWARD;
    }

    current_eval_context =
        EvaluationContext(for_current_state, current_g, true, &statistics);
    statistics.inc_evaluated_states();
    if (open_list->is_dead_end(current_eval_context)) continue;

    if (search_progress.check_progress(current_eval_context)) {
      statistics.print_checkpoint_line(current_g);
      reward_progress();
    }

    break;
  }

  return IN_PROGRESS;
}

SearchStatus LazySFBS::for_step() {
  vector<OperatorID> applicable_ops;
  statistics.inc_expanded();
  successor_generator.generate_applicable_ops(for_current_state,
                                              applicable_ops);

  ordered_set::OrderedSet<OperatorID> preferred_operators;

  for (const shared_ptr<Evaluator> &preferred_operator_evaluator :
       preferred_operator_evaluators) {
    collect_preferred_operators(current_eval_context,
                                preferred_operator_evaluator.get(),
                                preferred_operators);
  }

  SearchNode node = partial_state_search_space.get_node(for_current_state);

  for (OperatorID op_id : applicable_ops) {
    OperatorProxy op = task_proxy.get_operators()[op_id];
    if ((current_g + op.get_cost()) >= bound) continue;

    GlobalState succ_state =
        regression_state_registry.get_successor_state(for_current_state, op);

    statistics.inc_generated();
    bool is_preferred = preferred_operators.contains(op_id);

    SearchNode succ_node = partial_state_search_space.get_node(succ_state);

    if (directions[succ_state] == BACKWARD) {
      meet_set_plan(for_current_state, op_id, succ_state, true);
      return SOLVED;
    }

    directions[succ_state] = FORWARD;
    state_operator_id[succ_state] = op_id;

    if (succ_node.is_new()) succ_node.open(node, op, get_adjusted_cost(op));

    auto succ_id_pair = make_pair(succ_state.get_id().get_value(),
                                  bac_current_state.get_id().get_value());
    int succ_g = current_g + get_adjusted_cost(op);

    if (closed_list.find(succ_id_pair) == closed_list.end()) {
      n_steps[succ_state] = n_steps[for_current_state] + 1;
      bool is_preferred = preferred_operators.contains(op_id);
      EvaluationContext succ_eval_context(current_eval_context.get_cache(),
                                          succ_g, is_preferred, nullptr);

      open_list->insert(
          succ_eval_context,
          make_pair(succ_state.get_id(), bac_current_state.get_id()));

    } else if (succ_node.get_g() > current_g + get_adjusted_cost(op)) {
      n_steps[succ_state] =
          std::min(n_steps[for_current_state] + 1, n_steps[succ_state]);

      if (reopen_closed_nodes) {
        if (succ_node.is_closed()) {
          statistics.inc_reopened();
        }
        succ_node.reopen(node, op, get_adjusted_cost(op));

        bool is_preferred = preferred_operators.contains(op_id);
        EvaluationContext succ_eval_context(current_eval_context.get_cache(),
                                            succ_g, is_preferred, nullptr);
        open_list->insert(
            succ_eval_context,
            make_pair(succ_state.get_id(), bac_current_state.get_id()));
      } else {
        succ_node.update_parent(node, op, get_adjusted_cost(op));
      }
    }
  }

  return fetch_next_state();
}

SearchStatus LazySFBS::bac_step() {
  statistics.inc_expanded();
  vector<OperatorID> applicable_ops;
  regression_successor_generator.generate_applicable_ops(bac_current_state,
                                                         applicable_ops);

  ordered_set::OrderedSet<OperatorID> preferred_operators;

  for (const shared_ptr<Evaluator> &preferred_operator_evaluator :
       preferred_operator_evaluators) {
    collect_preferred_operators(current_eval_context,
                                preferred_operator_evaluator.get(),
                                preferred_operators);
  }

  SearchNode node = partial_state_search_space.get_node(bac_current_state);

  for (OperatorID op_id : applicable_ops) {
    OperatorProxy op = regression_task_proxy.get_operators()[op_id];
    if ((current_g + op.get_cost()) >= bound) continue;

    OperatorProxy forward_op = task_proxy.get_operators()[op_id];
    bool any = false;

    for (EffectProxy effect : forward_op.get_effects()) {
      FactPair effect_pair = effect.get_fact().get_pair();
      if (bac_current_state[effect_pair.var] == effect_pair.value) {
        any = true;
        break;
      }
    }

    if (!any) continue;

    StateID succ_id =
        regression_state_registry.get_predecessor_state(bac_current_state, op);

    if (succ_id == StateID::no_state) continue;

    GlobalState succ_state = regression_state_registry.lookup_state(succ_id);

    statistics.inc_generated();
    bool is_preferred = preferred_operators.contains(op_id);

    SearchNode succ_node = partial_state_search_space.get_node(succ_state);

    if (directions[succ_state] == FORWARD) {
      meet_set_plan(succ_state, op_id, bac_current_state, false);
      return SOLVED;
    }

    directions[succ_state] = BACKWARD;
    state_operator_id[succ_state] = op_id;

    if (succ_node.is_new()) succ_node.open(node, op, get_adjusted_cost(op));

    auto succ_id_pair = make_pair(for_current_state.get_id().get_value(),
                                  succ_state.get_id().get_value());
    int succ_g = current_g + get_adjusted_cost(op);

    if (closed_list.find(succ_id_pair) == closed_list.end()) {
      n_steps[succ_state] = n_steps[bac_current_state] + 1;
      bool is_preferred = preferred_operators.contains(op_id);
      EvaluationContext succ_eval_context(current_eval_context.get_cache(),
                                          succ_g, is_preferred, nullptr);

      open_list->insert(succ_eval_context, make_pair(for_current_state.get_id(),
                                                     succ_state.get_id()));
    } else if (succ_node.get_g() > current_g + get_adjusted_cost(op)) {
      n_steps[succ_state] =
          std::min(n_steps[bac_current_state] + 1, n_steps[succ_state]);

      if (reopen_closed_nodes) {
        if (succ_node.is_closed()) {
          statistics.inc_reopened();
        }
        succ_node.reopen(node, op, get_adjusted_cost(op));

        bool is_preferred = preferred_operators.contains(op_id);
        EvaluationContext succ_eval_context(current_eval_context.get_cache(),
                                            succ_g, is_preferred, nullptr);
        open_list->insert(
            succ_eval_context,
            make_pair(for_current_state.get_id(), succ_state.get_id()));
      } else {
        succ_node.update_parent(node, op, get_adjusted_cost(op));
      }
    }
  }

  return fetch_next_state();
}

void LazySFBS::meet_set_plan(const GlobalState &s_f, OperatorID op_id,
                             const GlobalState &s_b, bool forward) {
  Plan plan;
  partial_state_search_space.trace_path(s_f, plan);

  if (forward) plan.push_back(op_id);

  Plan regression_plan;
  partial_state_search_space.trace_path(s_b, regression_plan);

  if (!forward) regression_plan.push_back(op_id);

  reverse(regression_plan.begin(), regression_plan.end());
  plan.insert(plan.end(), regression_plan.begin(), regression_plan.end());
  set_plan(plan);
}

bool LazySFBS::check_goal_and_set_plan(const GlobalState &state) {
  if (task_properties::is_goal_state(task_proxy, state)) {
    Plan plan;
    partial_state_search_space.trace_path(state, plan);
    set_plan(plan);
    return true;

    cout << "#forward actions: " << plan.size() << endl;
    cout << "#backward actions: " << 0 << endl;
  }

  return false;
}

bool LazySFBS::check_initial_and_set_plan(const GlobalState &state) {
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
  cout << "#forward actions: " << 0 << endl;
  cout << "#backward actions: " << plan.size() << endl;

  return true;
}

bool LazySFBS::check_meeting_and_set_plan(const GlobalState &s_f,
                                          const GlobalState &s_b) {
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

void LazySFBS::print_statistics() const {
  statistics.print_detailed_statistics();
  partial_state_search_space.print_statistics();
}

void LazySFBS::reward_progress() { open_list->boost_preferred(); }

void add_options_to_parser(OptionParser &parser) {
  SearchEngine::add_pruning_option(parser);
  SearchEngine::add_options_to_parser(parser);
}
}  // namespace lazy_sfbs
