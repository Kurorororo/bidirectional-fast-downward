#include "eager_sfbs.h"

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

namespace eager_sfbs {
EagerSFBS::EagerSFBS(const Options &opts)
    : SearchEngine(opts),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      prune_goal(opts.get<bool>("prune_goal")),
      is_initial(true),
      open_list(opts.get<shared_ptr<FrontToFrontOpenListFactory>>("open")
                    ->create_frontier_open_list()),
      f_evaluator(opts.get<shared_ptr<Evaluator>>("f_eval", nullptr)),
      preferred_operator_evaluators(
          opts.get_list<shared_ptr<FrontToFrontHeuristic>>("preferred")),
      partial_state_task(tasks::PartialStateTask::get_partial_state_task()),
      partial_state_task_proxy(*partial_state_task),
      regression_state_registry(partial_state_task_proxy),
      partial_state_search_space(regression_state_registry),
      regression_task(tasks::RegressionTask::get_regression_task()),
      regression_task_proxy(*regression_task),
      regression_successor_generator(regression_task) {}

void EagerSFBS::initialize() {
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
  for (const shared_ptr<Evaluator> &evaluator : preferred_operator_evaluators) {
    evaluator->get_path_dependent_evaluators(evals);
  }

  /*
    Collect path-dependent evaluators that are used in the f_evaluator.
    They are usually also used in the open list and will hence already be
    included, but we want to be sure.
  */
  if (f_evaluator) {
    f_evaluator->get_path_dependent_evaluators(evals);
  }

  path_dependent_evaluators.assign(evals.begin(), evals.end());

  const GlobalState &initial_state =
      regression_state_registry.get_initial_state();
  for (Evaluator *evaluator : path_dependent_evaluators) {
    evaluator->notify_initial_state(initial_state);
  }
  is_forward[initial_state] = true;

  goal_state_values = regression_task->get_goal_state_values();
  vector<int> to_be_moved = goal_state_values;
  State goal_state = regression_task_proxy.create_state(move(to_be_moved));
  const GlobalState global_goal_state =
      regression_state_registry.create_goal_state(goal_state);
  is_forward[global_goal_state] = false;

  open_list->set_goal(global_goal_state);
  EvaluationContext eval_context(initial_state, 0, true, &statistics);

  statistics.inc_evaluated_states();

  if (open_list->is_dead_end(eval_context)) {
    cout << "Initial state is a dead end." << endl;
  } else {
    if (search_progress.check_progress(eval_context))
      statistics.print_checkpoint_line(0);
    start_f_value_statistics(eval_context);
    SearchNode n_f = partial_state_search_space.get_node(initial_state);
    n_f.open_initial();
    SearchNode n_b = partial_state_search_space.get_node(global_goal_state);
    n_b.open_initial();

    open_list->insert(eval_context, make_pair(initial_state.get_id(),
                                              global_goal_state.get_id()));
    n_steps[initial_state] = 0;
    n_steps[global_goal_state] = 0;
  }

  print_initial_evaluator_values(eval_context);
}

void EagerSFBS::print_statistics() const {
  statistics.print_detailed_statistics();
  partial_state_search_space.print_statistics();
}

SearchStatus EagerSFBS::step() {
  tl::optional<SearchNode> n_f;
  tl::optional<SearchNode> n_b;
  while (true) {
    if (open_list->empty()) {
      cout << "Completely explored state space -- no solution!" << endl;
      return FAILED;
    }
    pair<StateID, StateID> frontier = open_list->remove_min();
    // TODO is there a way we can avoid creating the state here and then
    //      recreate it outside of this function with node.get_state()?
    //      One way would be to store GlobalState objects inside SearchNodes
    //      instead of StateIDs

    pair<int, int> id_pair =
        make_pair(frontier.first.get_value(), frontier.second.get_value());

    if (closed_list.find(id_pair) != closed_list.end()) continue;

    closed_list.insert(id_pair);

    GlobalState s_f = regression_state_registry.lookup_state(frontier.first);
    GlobalState s_b = regression_state_registry.lookup_state(frontier.second);

    n_f.emplace(partial_state_search_space.get_node(s_f));
    n_b.emplace(partial_state_search_space.get_node(s_b));

    /*
      We can pass calculate_preferred=false here since preferred
      operators are computed when the state is expanded.
    */
    open_list->set_goal(s_b);
    EvaluationContext eval_context(s_f, n_f->get_g(), false, &statistics);

    assert(!n_f->is_dead_end());
    update_f_value_statistics(eval_context);
    statistics.inc_expanded();
    break;
  }

  GlobalState s_f = n_f->get_state();
  GlobalState s_b = n_b->get_state();

  if (check_meeting_and_set_plan(s_f, s_b)) return SOLVED;

  if (check_goal_and_set_plan(s_f)) return SOLVED;

  if (check_initial_and_set_plan(s_b)) return SOLVED;

  SearchStatus status = IN_PROGRESS;

  if (n_steps[s_f] <= n_steps[s_b]) {
    status = forward_step(n_f, n_b);
  } else {
    status = backward_step(n_f, n_b);
  }

  return status;
}

void EagerSFBS::reward_progress() {
  // Boost the "preferred operator" open lists somewhat whenever
  // one of the heuristics finds a state with a new best h value.
  open_list->boost_preferred();
}

void EagerSFBS::dump_search_space() const {
  partial_state_search_space.dump(partial_state_task_proxy);
}

void EagerSFBS::start_f_value_statistics(EvaluationContext &eval_context) {
  if (f_evaluator) {
    int f_value = eval_context.get_evaluator_value(f_evaluator.get());
    statistics.report_f_value_progress(f_value);
  }
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every
   state. */
void EagerSFBS::update_f_value_statistics(EvaluationContext &eval_context) {
  if (f_evaluator) {
    int f_value = eval_context.get_evaluator_value(f_evaluator.get());
    statistics.report_f_value_progress(f_value);
  }
}

SearchStatus EagerSFBS::forward_step(const tl::optional<SearchNode> &n_f,
                                     const tl::optional<SearchNode> &n_b) {
  GlobalState s_f = n_f->get_state();
  GlobalState s_b = n_b->get_state();

  vector<OperatorID> applicable_ops;
  successor_generator.generate_applicable_ops(s_f, applicable_ops);

  ordered_set::OrderedSet<OperatorID> preferred_operators;

  open_list->set_goal(s_b);
  EvaluationContext eval_context(s_f, n_f->get_g(), false, &statistics, true);

  for (const shared_ptr<Evaluator> &preferred_operator_evaluator :
       preferred_operator_evaluators) {
    collect_preferred_operators(
        eval_context, preferred_operator_evaluator.get(), preferred_operators);
  }

  for (OperatorID op_id : applicable_ops) {
    OperatorProxy op = task_proxy.get_operators()[op_id];
    if ((n_f->get_real_g() + op.get_cost()) >= bound) continue;

    GlobalState succ_state =
        regression_state_registry.get_successor_state(s_f, op);

    if (succ_state.get_id() == n_f->get_parent_state_id()) continue;

    statistics.inc_generated();
    bool is_preferred = preferred_operators.contains(op_id);

    SearchNode succ_node = partial_state_search_space.get_node(succ_state);

    if (!succ_node.is_new() && !is_forward[succ_state]) {
      meet_set_plan(s_f, op_id, succ_state, true);
      return SOLVED;
    }

    is_forward[succ_state] = true;

    for (Evaluator *evaluator : path_dependent_evaluators) {
      evaluator->notify_state_transition(s_f, op_id, succ_state);
    }

    if (succ_node.is_dead_end()) continue;

    if (succ_node.is_new()) succ_node.open(*n_f, op, get_adjusted_cost(op));

    auto succ_id_pair =
        make_pair(succ_state.get_id().get_value(), s_b.get_id().get_value());

    if (closed_list.find(succ_id_pair) == closed_list.end()) {
      int succ_g = n_f->get_g() + get_adjusted_cost(op);

      open_list->set_goal(s_b);
      EvaluationContext succ_eval_context(succ_state, succ_g, is_preferred,
                                          &statistics);
      statistics.inc_evaluated_states();

      if (open_list->is_dead_end(succ_eval_context)) {
        statistics.inc_dead_ends();
        continue;
      }

      n_steps[succ_state] = n_steps[s_f] + 1;

      open_list->insert(succ_eval_context,
                        make_pair(succ_state.get_id(), s_b.get_id()));

      if (search_progress.check_progress(succ_eval_context)) {
        statistics.print_checkpoint_line(succ_node.get_g());
        reward_progress();
      }
    } else if (succ_node.get_g() > n_f->get_g() + get_adjusted_cost(op)) {
      n_steps[succ_state] = std::min(n_steps[s_f] + 1, n_steps[succ_state]);

      if (reopen_closed_nodes) {
        if (succ_node.is_closed()) {
          statistics.inc_reopened();
        }
        succ_node.reopen(*n_f, op, get_adjusted_cost(op));

        open_list->set_goal(s_b);
        EvaluationContext pre_eval_context(succ_state, succ_node.get_g(),
                                           is_preferred, &statistics);
        open_list->insert(pre_eval_context,
                          make_pair(succ_state.get_id(), s_b.get_id()));
      } else {
        succ_node.update_parent(*n_f, op, get_adjusted_cost(op));
      }
    }
  }

  return IN_PROGRESS;
}

SearchStatus EagerSFBS::backward_step(const tl::optional<SearchNode> &n_f,
                                      const tl::optional<SearchNode> &n_b) {
  GlobalState s_f = n_f->get_state();
  GlobalState s_b = n_b->get_state();

  vector<OperatorID> applicable_ops;
  regression_successor_generator.generate_applicable_ops(s_b, applicable_ops);

  ordered_set::OrderedSet<OperatorID> preferred_operators;

  open_list->set_goal(s_b);
  EvaluationContext eval_context(s_f, n_f->get_g(), false, &statistics, true);

  for (const shared_ptr<Evaluator> &preferred_operator_evaluator :
       preferred_operator_evaluators) {
    collect_preferred_operators(
        eval_context, preferred_operator_evaluator.get(), preferred_operators);
  }

  bool do_predecessor_pruning = prune_goal && is_initial;
  if (is_initial) is_initial = false;

  for (OperatorID op_id : applicable_ops) {
    if (do_predecessor_pruning) {
      OperatorProxy fop = task_proxy.get_operators()[op_id];
      bool add_goal = false;

      for (EffectProxy effect : fop.get_effects()) {
        FactPair effect_pair = effect.get_fact().get_pair();
        if (goal_state_values[effect_pair.var] == effect_pair.value) {
          add_goal = true;
          break;
        }
      }

      if (!add_goal) continue;
    }

    OperatorProxy forward_op = task_proxy.get_operators()[op_id];
    bool any = false;

    for (EffectProxy effect : forward_op.get_effects()) {
      FactPair effect_pair = effect.get_fact().get_pair();
      if (s_b[effect_pair.var] == effect_pair.value) {
        any = true;
        break;
      }
    }

    if (!any) continue;

    OperatorProxy op = regression_task_proxy.get_operators()[op_id];
    if ((n_b->get_real_g() + op.get_cost()) >= bound) continue;

    StateID pre_state_id =
        regression_state_registry.get_predecessor_state(s_b, op);

    if (pre_state_id == StateID::no_state ||
        pre_state_id == n_b->get_parent_state_id())
      continue;

    GlobalState pre_state =
        regression_state_registry.lookup_state(pre_state_id);
    statistics.inc_generated();

    if (prune_goal && task_properties::is_goal_state(task_proxy, pre_state)) {
      continue;
    }

    bool is_preferred = preferred_operators.contains(op_id);

    SearchNode pre_node = partial_state_search_space.get_node(pre_state);

    if (!pre_node.is_new() && is_forward[pre_state]) {
      meet_set_plan(pre_state, op_id, s_b, false);
      return SOLVED;
    }

    is_forward[pre_state] = false;

    for (Evaluator *evaluator : path_dependent_evaluators) {
      evaluator->notify_state_transition(pre_state, op_id, s_b);
    }

    if (pre_node.is_dead_end()) continue;

    if (pre_node.is_new()) pre_node.open(*n_b, op, get_adjusted_cost(op));

    auto pre_id_pair =
        make_pair(s_f.get_id().get_value(), pre_state.get_id().get_value());

    if (closed_list.find(pre_id_pair) == closed_list.end()) {
      int pre_g = n_b->get_g() + get_adjusted_cost(op);

      open_list->set_goal(pre_state);
      EvaluationContext pre_eval_context(s_f, pre_g, is_preferred, &statistics);
      statistics.inc_evaluated_states();

      if (open_list->is_dead_end(pre_eval_context)) {
        statistics.inc_dead_ends();
        continue;
      }

      n_steps[pre_state] = n_steps[s_b] + 1;

      open_list->insert(pre_eval_context,
                        make_pair(s_f.get_id(), pre_state.get_id()));
      if (search_progress.check_progress(pre_eval_context)) {
        statistics.print_checkpoint_line(pre_node.get_g());
        reward_progress();
      }
    } else if (pre_node.get_g() > n_b->get_g() + get_adjusted_cost(op)) {
      n_steps[pre_state] = std::min(n_steps[s_b] + 1, n_steps[pre_state]);

      if (reopen_closed_nodes) {
        if (pre_node.is_closed()) {
          statistics.inc_reopened();
        }
        pre_node.reopen(*n_b, op, get_adjusted_cost(op));

        open_list->set_goal(pre_state);
        EvaluationContext pre_eval_context(s_f, pre_node.get_g(), is_preferred,
                                           &statistics);
        open_list->insert(pre_eval_context,
                          make_pair(s_f.get_id(), pre_state.get_id()));
      } else {
        pre_node.update_parent(*n_b, op, get_adjusted_cost(op));
      }
    }
  }

  return IN_PROGRESS;
}

void EagerSFBS::meet_set_plan(const GlobalState &s_f, OperatorID op_id,
                              const GlobalState &s_b, bool forward) {
  Plan plan;
  partial_state_search_space.trace_path(s_f, plan);

  if (forward) plan.push_back(op_id);

  cout << "#forward actions: " << plan.size() << endl;
  Plan regression_plan;
  partial_state_search_space.trace_path(s_b, regression_plan);

  if (!forward) regression_plan.push_back(op_id);

  reverse(regression_plan.begin(), regression_plan.end());
  cout << "#backward actions: " << regression_plan.size() << endl;
  plan.insert(plan.end(), regression_plan.begin(), regression_plan.end());
  set_plan(plan);
}

bool EagerSFBS::check_goal_and_set_plan(const GlobalState &state) {
  if (task_properties::is_goal_state(task_proxy, state)) {
    Plan plan;
    partial_state_search_space.trace_path(state, plan);
    set_plan(plan);
    return true;
  }

  return false;
}

bool EagerSFBS::check_initial_and_set_plan(const GlobalState &state) {
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

bool EagerSFBS::check_meeting_and_set_plan(const GlobalState &s_f,
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

void add_options_to_parser(OptionParser &parser) {
  SearchEngine::add_pruning_option(parser);
  SearchEngine::add_options_to_parser(parser);
}
}  // namespace eager_sfbs
