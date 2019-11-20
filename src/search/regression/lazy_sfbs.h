#ifndef LAZY_SFBS_H
#define LAZY_SFBS_H

#include "../bidirectional/bidirectional_search.h"
#include "../front_to_front/front_to_front_heuristic.h"
#include "../front_to_front/front_to_front_open_list.h"
#include "regression_state_registry.h"
#include "regression_successor_generator.h"
#include "regression_task.h"

#include <memory>
#include <optional.hh>
#include <vector>

class Evaluator;

namespace options {
class OptionParser;
class Options;
}  // namespace options

namespace lazy_sfbs {
class LazySFBS : public SearchEngine {
  enum Direction { NONE = 0, FORWARD = 1, BACKWARD = 2 };

  struct FrontierHash {
    std::size_t operator()(const std::pair<int, int> &entry) const {
      return std::hash<int>()(entry.first) ^ std::hash<int>()(entry.second);
    }
  };

  const bool reopen_closed_nodes;
  bool preferred_successors_first;
  bool is_initial;
  Direction current_direction;

  std::unique_ptr<FrontToFrontFrontierOpenList> open_list;
  std::shared_ptr<Evaluator> f_evaluator;

  std::vector<Evaluator *> path_dependent_evaluators;
  std::vector<std::shared_ptr<FrontToFrontHeuristic>>
      preferred_operator_evaluators;

  std::unordered_set<std::pair<int, int>, FrontierHash> closed_list;
  PerStateInformation<Direction> directions;
  PerStateInformation<OperatorID> state_operator_id;

  SearchStatus for_step();
  SearchStatus bac_step();

  void generate_successors();
  void generate_predecessors();
  SearchStatus fetch_next_state();

  std::vector<OperatorID> get_successor_operators(
      const ordered_set::OrderedSet<OperatorID> &preferred_operators) const;
  std::vector<OperatorID> get_predecessor_operators(
      const ordered_set::OrderedSet<OperatorID> &preferred_operators) const;

  void start_f_value_statistics(EvaluationContext &eval_context);
  void update_f_value_statistics(EvaluationContext &eval_context);
  void reward_progress();
  bool check_goal_and_set_plan(const GlobalState &state);
  bool check_initial_and_set_plan(const GlobalState &state);
  bool check_meeting_and_set_plan(const GlobalState &s_f,
                                  const GlobalState &s_b);
  void meet_set_plan(const GlobalState &s_f, OperatorID op_id,
                     const GlobalState &s_b, bool forward);

 protected:
  const std::shared_ptr<AbstractTask> partial_state_task;
  TaskProxy partial_state_task_proxy;
  RegressionStateRegistry regression_state_registry;
  SearchSpace partial_state_search_space;
  const std::shared_ptr<tasks::RegressionTask> regression_task;
  TaskProxy regression_task_proxy;
  regression_successor_generator::RegressionSuccessorGenerator
      regression_successor_generator;
  PerStateInformation<int> n_steps;

  GlobalState for_current_state;
  GlobalState bac_current_state;
  int current_g;
  EvaluationContext current_eval_context;

  virtual void initialize() override;
  virtual SearchStatus step() override;

 public:
  explicit LazySFBS(const options::Options &opts);
  virtual ~LazySFBS() = default;

  virtual void print_statistics() const override;

  void dump_search_space() const;
};

extern void add_options_to_parser(options::OptionParser &parser);
}  // namespace lazy_sfbs

#endif
