#ifndef BIDIRECTIONAL_EAGER_SEARCH_H
#define BIDIRECTIONAL_EAGER_SEARCH_H

#include <memory>
#include <optional.hh>
#include <vector>

#include "../front_to_front/front_to_front_heuristic.h"
#include "../front_to_front/front_to_front_open_list.h"
#include "../heuristics/max_heuristic.h"
#include "../operator_id.h"
#include "../search_engine.h"
#include "../search_progress.h"
#include "../search_space.h"
#include "regression_state_registry.h"
#include "regression_successor_generator.h"
#include "regression_task.h"
#include "symbolic_closed.h"

class Evaluator;

namespace options {
class OptionParser;
class Options;
}  // namespace options

namespace bidirectional_eager_search {
class BidirectionalEagerSearch : public SearchEngine {
  enum Direction { NONE = 0, FORWARD = 1, BACKWARD = 2 };

  const bool reopen_closed_nodes;
  bool prune_goal;
  bool is_initial;
  bool bdd;
  std::vector<int> goal_state_values;
  int initial_branching_f;
  int initial_branching_b;
  int sum_branching_f;
  int sum_branching_b;
  int expanded_f;
  int expanded_b;
  int d_node_value_f;
  int d_node_value_b;
  int max_steps;
  int steps;

  std::unordered_map<Direction, std::shared_ptr<FrontToFrontStateOpenList>>
      open_lists;
  std::unordered_map<Direction, std::shared_ptr<Evaluator>> f_evaluators;

  std::unordered_map<Direction, std::vector<Evaluator *>>
      path_dependent_evaluators;
  std::unordered_map<Direction,
                     std::vector<std::shared_ptr<FrontToFrontHeuristic>>>
      preferred_operator_evaluators;
  StateID d_node_f;
  StateID d_node_b;

  void start_f_value_statistics(Direction d, EvaluationContext &eval_context);
  void update_f_value_statistics(Direction d, EvaluationContext &eval_context);
  void reward_progress(Direction d);
  bool check_goal_and_set_plan(const GlobalState &state);
  bool check_initial_and_set_plan(const GlobalState &state);
  bool check_meeting_and_set_plan(const GlobalState &s_f,
                                  const GlobalState &s_b);
  void meet_set_plan(Direction d, const GlobalState &s_f, OperatorID op_id,
                     const GlobalState &s_b);
  SearchStatus forward_step(const tl::optional<SearchNode> &node);
  SearchStatus backward_step(const tl::optional<SearchNode> &node);
  void forward_reeval_all();
  void backward_reeval_all();

 protected:
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
  PerStateInformation<StateID> pair_states;
  Direction current_direction;
  PerStateInformation<Direction> directions;
  std::vector<StateID> bggs;
  std::shared_ptr<FrontToFrontHeuristic> bgg_eval;

  virtual void initialize() override;
  virtual SearchStatus step() override;

  bool check_meeting_and_set_plan(Direction d, const GlobalState &parent,
                                  OperatorID op_id, const GlobalState &state);

  StateID get_subsuming_state_id(const GlobalState &s) const;

  StateID get_subsumed_state_id(const GlobalState &s) const;

 public:
  enum DNodeType { FRONT_TO_END = 0, TTBS = 1, BGG = 2, MAX_G = 3 };
  enum ReevalMethod { NO = 0, NOT_SIMILAR = 1, ALL = 2 };
  DNodeType d_node_type;
  ReevalMethod reeval_method;

  explicit BidirectionalEagerSearch(const options::Options &opts);
  virtual ~BidirectionalEagerSearch() = default;

  virtual void print_statistics() const override;

  void dump_search_space() const;
};

extern void add_options_to_parser(options::OptionParser &parser);
}  // namespace bidirectional_eager_search

#endif
