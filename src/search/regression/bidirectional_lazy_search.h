#ifndef SEARCH_ENGINES_BIDIRECTIONAL_LAZY_SEARCH_H
#define SEARCH_ENGINES_BIDIRECTIONAL_LAZY_SEARCH_H

#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../global_state.h"
#include "../open_list.h"
#include "../operator_id.h"
#include "../search_engine.h"
#include "../search_progress.h"
#include "../search_space.h"

#include "../front_to_front/front_to_front_heuristic.h"
#include "../front_to_front/front_to_front_open_list.h"
#include "regression_state_registry.h"
#include "regression_successor_generator.h"
#include "regression_task.h"
#include "symbolic_closed.h"

#include "../utils/rng.h"

#include <memory>
#include <vector>

namespace options {
class Options;
}

namespace bidirectional_lazy_search {
class BidirectionalLazySearch : public SearchEngine {
  enum Direction { NONE = 0, FORWARD = 1, BACKWARD = 2 };

 protected:
  std::unique_ptr<FrontToFrontEdgeOpenList> for_open_list;
  std::unique_ptr<FrontToFrontEdgeOpenList> bac_open_list;

  // Search behavior parameters
  bool reopen_closed_nodes;  // whether to reopen closed nodes upon finding
                             // lower g paths
  bool randomize_successors;
  bool preferred_successors_first;
  bool prune_goal;
  bool bdd;
  bool front_to_front;
  bool reeval;
  bool use_bgg;
  int h_min_bgg;
  StateID arg_min_bgg;
  std::shared_ptr<utils::RandomNumberGenerator> rng;

  std::vector<Evaluator *> for_path_dependent_evaluators;
  std::vector<std::shared_ptr<FrontToFrontHeuristic>> for_preferred_operator_evaluators;
  std::vector<Evaluator *> bac_path_dependent_evaluators;
  std::vector<std::shared_ptr<FrontToFrontHeuristic>> bac_preferred_operator_evaluators;

  const std::shared_ptr<AbstractTask> partial_state_task;
  TaskProxy partial_state_task_proxy;
  RegressionStateRegistry regression_state_registry;
  SearchSpace partial_state_search_space;
  const std::shared_ptr<tasks::RegressionTask> regression_task;
  TaskProxy regression_task_proxy;
  regression_successor_generator::RegressionSuccessorGenerator
      regression_successor_generator;
  symbolic_closed::SymbolicClosedList for_symbolic_closed_list;
  symbolic_closed::SymbolicClosedList bac_symbolic_closed_list;
  Direction current_direction;
  PerStateInformation<Direction> directions;
  PerStateInformation<StateID> pair_state;

  GlobalState for_current_state;
  StateID for_current_predecessor_id;
  OperatorID for_current_operator_id;
  int for_current_g;
  int for_current_real_g;
  EvaluationContext for_current_eval_context;

  GlobalState bac_current_state;
  StateID bac_current_successor_id;
  OperatorID bac_current_operator_id;
  int bac_current_g;
  int bac_current_real_g;
  EvaluationContext bac_current_eval_context;

  EdgeOpenListEntry previous_entry;
  StateID parent_id;
  EvaluationContext parent_eval_context;
  std::vector<StateID> bggs;
  std::shared_ptr<FrontToFrontHeuristic> bgg_eval;

  virtual void initialize() override;
  virtual SearchStatus step() override;

  SearchStatus for_step();
  SearchStatus bac_step();

  void generate_successors();
  void generate_predecessors();
  SearchStatus for_fetch_next_state();
  SearchStatus bac_fetch_next_state();

  void for_reward_progress();
  void bac_reward_progress();

  std::vector<OperatorID> get_successor_operators(
      const ordered_set::OrderedSet<OperatorID> &preferred_operators) const;
  std::vector<OperatorID> get_predecessor_operators(
      const ordered_set::OrderedSet<OperatorID> &preferred_operators) const;

  bool check_goal_and_set_plan(const GlobalState &state);
  bool check_initial_and_set_plan(const GlobalState &state);
  bool check_meeting_and_set_plan(const GlobalState &s_f,
                                  const GlobalState &s_b);
  void meet_set_plan(Direction d, const GlobalState &s_f, OperatorID op_id,
                     const GlobalState &s_b);

  StateID get_subsuming_state_id(const GlobalState &s) const;

  StateID get_subsumed_state_id(const GlobalState &s) const;

 public:
  explicit BidirectionalLazySearch(const options::Options &opts);
  virtual ~BidirectionalLazySearch() = default;

  void for_set_preferred_operator_evaluators(
      std::vector<std::shared_ptr<FrontToFrontHeuristic>> &evaluators);
  void bac_set_preferred_operator_evaluators(
      std::vector<std::shared_ptr<FrontToFrontHeuristic>> &evaluators);

  virtual void print_statistics() const override;
};
}  // namespace bidirectional_lazy_search

#endif
