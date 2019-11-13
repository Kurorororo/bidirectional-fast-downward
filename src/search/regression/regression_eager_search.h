#ifndef REGRESSION_EAGER_SEARCH_H
#define REGRESSION_EAGER_SEARCH_H

#include "../abstract_task.h"
#include "../front_to_front/front_to_front_heuristic.h"
#include "../front_to_front/front_to_front_open_list.h"
#include "../open_list.h"
#include "../search_engine.h"
#include "../search_space.h"
#include "../task_proxy.h"
#include "partial_state_task.h"
#include "regression_state_registry.h"
#include "regression_successor_generator.h"
#include "regression_task.h"
#include "symbolic_closed.h"

#include <memory>
#include <vector>

class Evaluator;
class PruningMethod;

namespace options {
class OptionParser;
class Options;
}  // namespace options

namespace regression_eager_search {
class RegressionEagerSearch : public SearchEngine {
  const bool reopen_closed_nodes;
  bool prune_goal;
  bool is_initial;
  bool bdd;

  std::shared_ptr<FrontToFrontStateOpenList> open_list;
  std::shared_ptr<Evaluator> f_evaluator;

  std::vector<Evaluator *> path_dependent_evaluators;
  std::vector<std::shared_ptr<FrontToFrontHeuristic>>
      preferred_operator_evaluators;

  std::vector<int> goal_state_values;

  void start_f_value_statistics(EvaluationContext &eval_context);
  void update_f_value_statistics(EvaluationContext &eval_context);
  void reward_progress();

 protected:
  const std::shared_ptr<AbstractTask> partial_state_task;
  TaskProxy partial_state_task_proxy;
  RegressionStateRegistry regression_state_registry;
  SearchSpace partial_state_search_space;
  const std::shared_ptr<tasks::RegressionTask> regression_task;
  TaskProxy regression_task_proxy;
  regression_successor_generator::RegressionSuccessorGenerator
      regression_successor_generator;
  symbolic_closed::SymbolicClosedList symbolic_closed_list;

  virtual void initialize() override;
  virtual SearchStatus step() override;

 public:
  explicit RegressionEagerSearch(const options::Options &opts);
  virtual ~RegressionEagerSearch() = default;

  virtual void print_statistics() const override;

  void dump_search_space() const;
};

extern void add_options_to_parser(options::OptionParser &parser);
}  // namespace regression_eager_search

#endif
