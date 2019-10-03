#ifndef REGRESSION_LAZY_SEARCH_H
#define REGRESSION_LAZY_SEARCH_H

#include "../evaluation_context.h"
#include "../front_to_front/front_to_front_heuristic.h"
#include "../front_to_front/front_to_front_open_list.h"
#include "../global_state.h"
#include "../operator_id.h"
#include "../search_engine.h"
#include "../search_progress.h"
#include "../task_proxy.h"
#include "regression_state_registry.h"
#include "regression_successor_generator.h"
#include "regression_task.h"

#include "../utils/rng.h"

#include <memory>
#include <vector>

namespace options {
class Options;
}

namespace regression_lazy_search {
class RegressionLazySearch : public SearchEngine {
 protected:
  std::unique_ptr<FrontToFrontEdgeOpenList> open_list;
  const std::shared_ptr<AbstractTask> partial_state_task;
  TaskProxy partial_state_task_proxy;
  RegressionStateRegistry regression_state_registry;
  SearchSpace partial_state_search_space;
  const std::shared_ptr<tasks::RegressionTask> regression_task;
  TaskProxy regression_task_proxy;
  regression_successor_generator::RegressionSuccessorGenerator
      regression_successor_generator;

  // Search behavior parameters
  bool reopen_closed_nodes;  // whether to reopen closed nodes upon finding
                             // lower g paths
  bool randomize_successors;
  bool preferred_successors_first;
  std::shared_ptr<utils::RandomNumberGenerator> rng;

  std::vector<Evaluator *> path_dependent_evaluators;
  std::vector<std::shared_ptr<Evaluator>> preferred_operator_evaluators;

  GlobalState current_state;
  StateID current_predecessor_id;
  OperatorID current_operator_id;
  int current_g;
  int current_real_g;
  EvaluationContext current_eval_context;

  virtual void initialize() override;
  virtual SearchStatus step() override;

  void generate_successors();
  SearchStatus fetch_next_state();

  void reward_progress();

  std::vector<OperatorID> get_successor_operators(
      const ordered_set::OrderedSet<OperatorID> &preferred_operators) const;

  bool check_initial_and_set_plan(const GlobalState &state);

 public:
  explicit RegressionLazySearch(const options::Options &opts);
  virtual ~RegressionLazySearch() = default;

  void set_preferred_operator_evaluators(
      std::vector<std::shared_ptr<Evaluator>> &evaluators);

  virtual void print_statistics() const override;
};
}  // namespace regression_lazy_search

#endif
