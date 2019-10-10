#ifndef EAGER_SFBS_H
#define EAGER_SFBS_H

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

namespace eager_sfbs {
class EagerSFBS : public SearchEngine {
  struct FrontierHash {
    std::size_t operator()(const std::pair<int, int> &entry) const {
      return std::hash<int>()(entry.first) ^ std::hash<int>()(entry.second);
    }
  };

  const bool reopen_closed_nodes;
  bool prune_goal;
  bool is_initial;
  std::vector<int> goal_state_values;

  std::unique_ptr<FrontToFrontFrontierOpenList> open_list;
  std::shared_ptr<Evaluator> f_evaluator;

  std::vector<Evaluator *> path_dependent_evaluators;
  std::vector<std::shared_ptr<FrontToFrontHeuristic>>
      preferred_operator_evaluators;

  std::unordered_set<std::pair<int, int>, FrontierHash> closed_list;
  PerStateInformation<bool> is_forward;

  void start_f_value_statistics(EvaluationContext &eval_context);
  void update_f_value_statistics(EvaluationContext &eval_context);
  void reward_progress();
  void meet_set_plan(const GlobalState &s_f, OperatorID op_id,
                     const GlobalState &s_b, bool forward);
  bool check_goal_and_set_plan(const GlobalState &state);
  bool check_initial_and_set_plan(const GlobalState &state);
  bool check_meeting_and_set_plan(const GlobalState &s_f,
                                  const GlobalState &s_b);
  SearchStatus forward_step(const tl::optional<SearchNode> &n_f,
                            const tl::optional<SearchNode> &n_b);
  SearchStatus backward_step(const tl::optional<SearchNode> &n_f,
                             const tl::optional<SearchNode> &n_b);

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

  virtual void initialize() override;
  virtual SearchStatus step() override;

 public:
  explicit EagerSFBS(const options::Options &opts);
  virtual ~EagerSFBS() = default;

  virtual void print_statistics() const override;

  void dump_search_space() const;
};

extern void add_options_to_parser(options::OptionParser &parser);
}  // namespace eager_sfbs

#endif
