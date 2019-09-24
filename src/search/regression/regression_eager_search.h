#ifndef REGRESSION_EAGER_SEARCH_H
#define REGRESSION_EAGER_SEARCH_H

#include "../abstract_task.h"
#include "../open_list.h"
#include "../search_engine.h"
#include "../task_proxy.h"
#include "regression_successor_generator.h"
#include "regression_task.h"

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

  std::unique_ptr<StateOpenList> open_list;
  std::shared_ptr<Evaluator> f_evaluator;

  std::vector<Evaluator *> path_dependent_evaluators;
  std::vector<std::shared_ptr<Evaluator>> preferred_operator_evaluators;
  std::shared_ptr<Evaluator> lazy_evaluator;

  std::shared_ptr<PruningMethod> pruning_method;

  void start_f_value_statistics(EvaluationContext &eval_context);
  void update_f_value_statistics(EvaluationContext &eval_context);
  void reward_progress();

 protected:
  const std::shared_ptr<tasks::RegressionTask> regression_task;
  TaskProxy regression_task_proxy;
  regression_successor_generator::RegressionSuccessorGenerator regression_successor_generator;

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
