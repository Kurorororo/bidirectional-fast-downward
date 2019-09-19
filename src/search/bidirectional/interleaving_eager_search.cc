#include "interleaving_eager_search.h"

#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../open_list_factory.h"
#include "../option_parser.h"
#include "../pruning_method.h"

#include "../algorithms/ordered_set.h"
#include "../task_utils/successor_generator.h"

#include "../utils/logging.h"

#include <cassert>
#include <cstdlib>
#include <memory>
#include <optional.hh>
#include <set>

using namespace std;

namespace interleaving_eager_search {
InterleavingEagerSearch::InterleavingEagerSearch(const Options &opts)
    : BidirectionalSearch(opts),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      current_direction(Direction::FORWARD) {
  open_lists[Direction::FORWARD] =
      opts.get<shared_ptr<OpenListFactory>>("open_f")->create_state_open_list();
  open_lists[Direction::BACKWARD] =
      opts.get<shared_ptr<OpenListFactory>>("open_b")->create_state_open_list();
  f_evaluators[Direction::FORWARD] =
      opts.get<shared_ptr<Evaluator>>("f_eval_f", nullptr);
  f_evaluators[Direction::BACKWARD] =
      opts.get<shared_ptr<Evaluator>>("f_eval_b", nullptr);
  preferred_operator_evaluators[Direction::FORWARD] =
      opts.get_list<shared_ptr<Evaluator>>("preferred_f");
  preferred_operator_evaluators[Direction::BACKWARD] =
      opts.get_list<shared_ptr<Evaluator>>("preferred_b");
  pruning_methods[Direction::FORWARD] =
      opts.get<shared_ptr<PruningMethod>>("pruning");
  pruning_methods[Direction::BACKWARD] =
      opts.get<shared_ptr<PruningMethod>>("pruning");
}

void InterleavingEagerSearch::initialize() {
  cout << "Conducting best first search"
       << (reopen_closed_nodes ? " with" : " without")
       << " reopening closed nodes, (real) bound = " << bound << endl;
  assert(open_lists[Direction::FORWARD]);
  assert(open_lists[Direction::BACKWARD]);

  // for (auto inverse_op : inverse_task_proxy.get_operators()) {
  //  OperatorProxy op = task_proxy.get_operators()[inverse_op.get_id()];

  //  cout << endl << op.get_name() << endl;

  //  cout << "forward preconditions" << endl;

  //  for (auto f : op.get_preconditions()) {
  //    cout << f.get_name() << endl;
  //  }

  //  cout << "backward preconditions" << endl;

  //  for (auto f : inverse_op.get_preconditions()) {
  //    cout << f.get_name() << endl;
  //  }

  //  cout << "forward effects" << endl;

  //  for (auto f : op.get_effects()) {
  //    cout << f.get_fact().get_name() << endl;
  //  }

  //  cout << "backward effects" << endl;

  //  for (auto f : inverse_op.get_effects()) {
  //    cout << f.get_fact().get_name() << endl;
  //  }
  //}

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

  const GlobalState &initial_state = state_registry.get_initial_state();
  for (Evaluator *evaluator : path_dependent_evaluators[Direction::FORWARD]) {
    evaluator->notify_initial_state(initial_state);
  }
  directions[initial_state] = Direction::FORWARD;

  State goal_state = inverse_task_proxy.get_initial_state();
  GlobalState global_goal_state = state_registry.create_goal_state(goal_state);
  for (Evaluator *evaluator : path_dependent_evaluators[Direction::BACKWARD]) {
    evaluator->notify_initial_state(global_goal_state);
  }
  directions[global_goal_state] = Direction::BACKWARD;

  /*
    Note: we consider the initial state as reached by a preferred
    operator.
  */
  EvaluationContext eval_context_f(initial_state, 0, true, &statistics);

  statistics.inc_evaluated_states();

  if (open_lists[Direction::FORWARD]->is_dead_end(eval_context_f)) {
    cout << "Initial state is a dead end." << endl;
  } else {
    if (search_progress.check_progress(eval_context_f)) {
      statistics.print_checkpoint_line(0);
    }
    start_f_value_statistics(Direction::FORWARD, eval_context_f);
    SearchNode node_f = search_space.get_node(initial_state);
    node_f.open_initial();

    open_lists[Direction::FORWARD]->insert(eval_context_f,
                                           initial_state.get_id());
  }

  EvaluationContext eval_context_b(global_goal_state, 0, true, &statistics);

  statistics.inc_evaluated_states();

  if (search_progress.check_progress(eval_context_b)) {
    statistics.print_checkpoint_line(0);
  }
  start_f_value_statistics(Direction::BACKWARD, eval_context_b);
  SearchNode node_b = search_space.get_node(global_goal_state);
  node_b.open_initial();

  open_lists[Direction::BACKWARD]->insert(eval_context_b,
                                          global_goal_state.get_id());

  print_initial_evaluator_values(eval_context_f);
  print_initial_evaluator_values(eval_context_b);

  pruning_methods[Direction::FORWARD]->initialize(task);
  pruning_methods[Direction::BACKWARD]->initialize(inverse_task);
}

void InterleavingEagerSearch::print_statistics() const {
  statistics.print_detailed_statistics();
  search_space.print_statistics();
  pruning_methods.at(Direction::FORWARD)->print_statistics();
  pruning_methods.at(Direction::BACKWARD)->print_statistics();
}

SearchStatus InterleavingEagerSearch::step() {
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
    GlobalState s = state_registry.lookup_state(id);
    node.emplace(search_space.get_node(s));

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

  GlobalState s = node->get_state();
  if (current_direction == Direction::FORWARD && check_goal_and_set_plan(s)) {
    cout << "#forward actions: " << get_plan().size() << endl;
    cout << "#backward actions: " << 0 << endl;
    return SOLVED;
  }

  vector<OperatorID> applicable_ops;

  if (current_direction == Direction::FORWARD)
    successor_generator.generate_applicable_ops(s, applicable_ops);
  else
    inverse_successor_generator.generate_applicable_ops(s, applicable_ops);

  /*
    TODO: When preferred operators are in use, a preferred operator will be
    considered by the preferred operator queues even when it is pruned.
  */
  pruning_methods[current_direction]->prune_operators(s, applicable_ops);

  // This evaluates the expanded state (again) to get preferred ops
  EvaluationContext eval_context(s, node->get_g(), false, &statistics, true);
  ordered_set::OrderedSet<OperatorID> preferred_operators;
  for (const shared_ptr<Evaluator> &preferred_operator_evaluator :
       preferred_operator_evaluators[current_direction]) {
    collect_preferred_operators(
        eval_context, preferred_operator_evaluator.get(), preferred_operators);
  }

  for (OperatorID op_id : applicable_ops) {
    OperatorProxy op = current_direction == Direction::FORWARD
                           ? task_proxy.get_operators()[op_id]
                           : inverse_task_proxy.get_operators()[op_id];
    if ((node->get_real_g() + op.get_cost()) >= bound) continue;

    GlobalState succ_state = state_registry.get_successor_state(s, op);
    statistics.inc_generated();
    bool is_preferred = preferred_operators.contains(op_id);

    SearchNode succ_node = search_space.get_node(succ_state);

    for (Evaluator *evaluator : path_dependent_evaluators[current_direction]) {
      evaluator->notify_state_transition(s, op_id, succ_state);
    }

    if (check_meeting_and_set_plan(current_direction, s, op_id, succ_state))
      return SOLVED;

    // Previously encountered dead end. Don't re-evaluate.
    if (succ_node.is_dead_end()) continue;

    if (succ_node.is_new()) {
      // We have not seen this state before.
      // Evaluate and create a new node.

      // Careful: succ_node.get_g() is not available here yet,
      // hence the stupid computation of succ_g.
      // TODO: Make this less fragile.
      int succ_g = node->get_g() + get_adjusted_cost(op);

      EvaluationContext succ_eval_context(succ_state, succ_g, is_preferred,
                                          &statistics);
      statistics.inc_evaluated_states();

      if (open_lists[current_direction]->is_dead_end(succ_eval_context)) {
        succ_node.mark_as_dead_end();
        statistics.inc_dead_ends();
        continue;
      }
      succ_node.open(*node, op, get_adjusted_cost(op));
      directions[succ_state] = current_direction;

      open_lists[current_direction]->insert(succ_eval_context,
                                            succ_state.get_id());
      if (search_progress.check_progress(succ_eval_context)) {
        statistics.print_checkpoint_line(succ_node.get_g());
        reward_progress(current_direction);
      }
    } else if (succ_node.get_g() > node->get_g() + get_adjusted_cost(op)) {
      // We found a new cheapest path to an open or closed state.
      if (reopen_closed_nodes) {
        if (succ_node.is_closed()) {
          /*
            TODO: It would be nice if we had a way to test
            that reopening is expected behaviour, i.e., exit
            with an error when this is something where
            reopening should not occur (e.g. A* with a
            consistent heuristic).
          */
          statistics.inc_reopened();
        }
        succ_node.reopen(*node, op, get_adjusted_cost(op));

        EvaluationContext succ_eval_context(succ_state, succ_node.get_g(),
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
        open_lists[current_direction]->insert(succ_eval_context,
                                              succ_state.get_id());
      } else {
        // If we do not reopen closed nodes, we just update the parent
        // pointers. Note that this could cause an incompatibility between the
        // g-value and the actual path that is traced back.
        succ_node.update_parent(*node, op, get_adjusted_cost(op));
      }
    }
  }

  current_direction = current_direction == Direction::FORWARD
                          ? Direction::BACKWARD
                          : Direction::FORWARD;

  return IN_PROGRESS;
}

void InterleavingEagerSearch::reward_progress(Direction d) {
  // Boost the "preferred operator" open lists somewhat whenever
  // one of the heuristics finds a state with a new best h value.

  open_lists[d]->boost_preferred();
}

void InterleavingEagerSearch::dump_search_space() const {
  search_space.dump(task_proxy);
}

void InterleavingEagerSearch::start_f_value_statistics(
    Direction d, EvaluationContext &eval_context) {
  if (f_evaluators[d]) {
    int f_value = eval_context.get_evaluator_value(f_evaluators[d].get());
    statistics.report_f_value_progress(f_value);
  }
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every
   state. */
void InterleavingEagerSearch::update_f_value_statistics(
    Direction d, EvaluationContext &eval_context) {
  if (f_evaluators[d]) {
    int f_value = eval_context.get_evaluator_value(f_evaluators[d].get());
    statistics.report_f_value_progress(f_value);
  }
}

void add_options_to_parser(OptionParser &parser) {
  SearchEngine::add_pruning_option(parser);
  SearchEngine::add_options_to_parser(parser);
}
}  // namespace interleaving_eager_search
