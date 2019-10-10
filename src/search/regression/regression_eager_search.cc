#include "regression_eager_search.h"

#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../open_list_factory.h"
#include "../option_parser.h"
#include "../pruning_method.h"

#include "../algorithms/ordered_set.h"
#include "../front_to_front/front_to_front_open_list_factory.h"
#include "../task_utils/task_properties.h"
#include "regression_successor_generator.h"

#include "../utils/logging.h"

#include <cassert>
#include <cstdlib>
#include <memory>
#include <optional.hh>
#include <set>

using namespace std;

namespace regression_eager_search {
RegressionEagerSearch::RegressionEagerSearch(const Options &opts)
    : SearchEngine(opts),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      prune_goal(opts.get<bool>("prune_goal")),
      is_initial(true),
      open_list(opts.get<shared_ptr<FrontToFrontOpenListFactory>>("open")
                    ->create_state_open_list()),
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

void RegressionEagerSearch::initialize() {
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

  goal_state_values = regression_task->get_goal_state_values();
  vector<int> to_be_moved = goal_state_values;
  State goal_state = partial_state_task_proxy.create_state(move(to_be_moved));
  const GlobalState global_goal_state =
      regression_state_registry.create_goal_state(goal_state);

  open_list->set_goal(global_goal_state);
  EvaluationContext eval_context(initial_state, 0, true, &statistics);

  statistics.inc_evaluated_states();

  if (open_list->is_dead_end(eval_context)) {
    cout << "Initial state is a dead end." << endl;
  } else {
    if (search_progress.check_progress(eval_context))
      statistics.print_checkpoint_line(0);
    start_f_value_statistics(eval_context);
    SearchNode node = partial_state_search_space.get_node(global_goal_state);
    node.open_initial();

    open_list->insert(eval_context, global_goal_state.get_id());
  }

  print_initial_evaluator_values(eval_context);
}

void RegressionEagerSearch::print_statistics() const {
  statistics.print_detailed_statistics();
  partial_state_search_space.print_statistics();
}

SearchStatus RegressionEagerSearch::step() {
  tl::optional<SearchNode> node;
  while (true) {
    if (open_list->empty()) {
      cout << "Completely explored state space -- no solution!" << endl;
      return FAILED;
    }
    StateID id = open_list->remove_min();
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
    update_f_value_statistics(eval_context);
    statistics.inc_expanded();
    break;
  }

  GlobalState s = node->get_state();

  const GlobalState &initial_state =
      regression_state_registry.get_initial_state();
  VariablesProxy variables = partial_state_task_proxy.get_variables();
  bool path_found = true;

  for (auto var : variables) {
    if (s[var.get_id()] != var.get_domain_size() - 1 &&
        s[var.get_id()] != initial_state[var.get_id()]) {
      path_found = false;
      break;
    }
  }

  if (path_found) {
    Plan plan;
    partial_state_search_space.trace_path(s, plan);
    reverse(plan.begin(), plan.end());
    set_plan(plan);
    return SOLVED;
  }

  vector<OperatorID> applicable_ops;
  regression_successor_generator.generate_applicable_ops(s, applicable_ops);

  open_list->set_goal(s);
  EvaluationContext eval_context(initial_state, node->get_g(), false,
                                 &statistics, true);
  ordered_set::OrderedSet<OperatorID> preferred_operators;
  for (const shared_ptr<Evaluator> &preferred_operator_evaluator :
       preferred_operator_evaluators) {
    collect_preferred_operators(
        eval_context, preferred_operator_evaluator.get(), preferred_operators);
  }

  // if (prune_goal && !is_initial &&
  //    task_properties::is_goal_state(task_proxy, s)) {
  //  return IN_PROGRESS;
  //}

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

    OperatorProxy op = regression_task_proxy.get_operators()[op_id];
    if ((node->get_real_g() + op.get_cost()) >= bound) continue;

    StateID pre_state_id =
        regression_state_registry.get_predecessor_state(s, op);

    if (pre_state_id == StateID::no_state) continue;

    GlobalState pre_state =
        regression_state_registry.lookup_state(pre_state_id);
    statistics.inc_generated();
    bool is_preferred = preferred_operators.contains(op_id);

    SearchNode pre_node = partial_state_search_space.get_node(pre_state);

    for (Evaluator *evaluator : path_dependent_evaluators) {
      evaluator->notify_state_transition(pre_state, op_id, s);
    }

    // Previously encountered dead end. Don't re-evaluate.
    if (pre_node.is_dead_end()) continue;

    if (pre_node.is_new()) {
      // We have not seen this state before.
      // Evaluate and create a new node.

      // Careful: pre_node.get_g() is not available here yet,
      // hence the stupid computation of pre_g.
      // TODO: Make this less fragile.
      int pre_g = node->get_g() + get_adjusted_cost(op);

      open_list->set_goal(pre_state);
      EvaluationContext pre_eval_context(initial_state, pre_g, is_preferred,
                                         &statistics);
      statistics.inc_evaluated_states();

      if (open_list->is_dead_end(pre_eval_context)) {
        pre_node.mark_as_dead_end();
        statistics.inc_dead_ends();
        continue;
      }
      pre_node.open(*node, op, get_adjusted_cost(op));

      open_list->insert(pre_eval_context, pre_state.get_id());
      if (search_progress.check_progress(pre_eval_context)) {
        statistics.print_checkpoint_line(pre_node.get_g());
        reward_progress();
      }
    } else if (pre_node.get_g() > node->get_g() + get_adjusted_cost(op)) {
      // We found a new cheapest path to an open or closed state.
      if (reopen_closed_nodes) {
        if (pre_node.is_closed()) {
          /*
            TODO: It would be nice if we had a way to test
            that reopening is expected behaviour, i.e., exit
            with an error when this is something where
            reopening should not occur (e.g. A* with a
            consistent heuristic).
          */
          statistics.inc_reopened();
        }
        pre_node.reopen(*node, op, get_adjusted_cost(op));

        open_list->set_goal(pre_state);
        EvaluationContext pre_eval_context(initial_state, pre_node.get_g(),
                                           is_preferred, &statistics);

        /*
          Note: our old code used to retrieve the h value from
          the search node here. Our new code recomputes it as
          necessary, thus avoiding the incredible ugliness of
          the old "set_evaluator_value" approach, which also
          did not generalize properly to settings with more
          than one evaluator.

          Reopening should not happen all that frequently, so
          the performance impact of this is hopefully not that
          large. In the medium term, we want the evaluators to
          remember evaluator values for states themselves if
          desired by the user, so that such recomputations
          will just involve a look-up by the Evaluator object
          rather than a recomputation of the evaluator value
          from scratch.
        */
        open_list->insert(pre_eval_context, pre_state.get_id());
      } else {
        // If we do not reopen closed nodes, we just update the parent pointers.
        // Note that this could cause an incompatibility between
        // the g-value and the actual path that is traced back.
        pre_node.update_parent(*node, op, get_adjusted_cost(op));
      }
    }
  }

  return IN_PROGRESS;
}

void RegressionEagerSearch::reward_progress() {
  // Boost the "preferred operator" open lists somewhat whenever
  // one of the heuristics finds a state with a new best h value.
  open_list->boost_preferred();
}

void RegressionEagerSearch::dump_search_space() const {
  partial_state_search_space.dump(partial_state_task_proxy);
}

void RegressionEagerSearch::start_f_value_statistics(
    EvaluationContext &eval_context) {
  if (f_evaluator) {
    int f_value = eval_context.get_evaluator_value(f_evaluator.get());
    statistics.report_f_value_progress(f_value);
  }
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every
   state. */
void RegressionEagerSearch::update_f_value_statistics(
    EvaluationContext &eval_context) {
  if (f_evaluator) {
    int f_value = eval_context.get_evaluator_value(f_evaluator.get());
    statistics.report_f_value_progress(f_value);
  }
}

void add_options_to_parser(OptionParser &parser) {
  SearchEngine::add_pruning_option(parser);
  SearchEngine::add_options_to_parser(parser);
}
}  // namespace regression_eager_search
