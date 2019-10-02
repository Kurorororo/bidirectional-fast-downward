#ifndef FRONT_TO_FRONT_HEURISTIC_H_
#define FRONT_TO_FRONT_HEURISTIC_H_

#include "../abstract_task.h"
#include "../evaluation_context.h"
#include "../evaluation_result.h"
#include "../evaluator.h"
#include "../task_proxy.h"

#include "../algorithms/ordered_set.h"

#include <memory>

namespace options {
class OptionParser;
class Options;
}  // namespace options

class FrontToFrontHeuristic : public Evaluator {
  ordered_set::OrderedSet<OperatorID> preferred_operators;

 protected:
  bool cache_goal;
  bool partial_state;
  const std::shared_ptr<AbstractTask> task;
  TaskProxy task_proxy;
  std::vector<std::pair<int, int>> current_goal;

  enum { DEAD_END = -1, NO_VALUE = -2 };

  virtual int compute_heuristic(const GlobalState &state) = 0;

  void set_preferred(const OperatorProxy &op);

  State convert_global_state(const GlobalState &global_state) const;

 public:
  FrontToFrontHeuristic();

  explicit FrontToFrontHeuristic(const options::Options &opts);
  virtual ~FrontToFrontHeuristic() override;

  virtual void set_goal(const GlobalState &state);

  virtual void get_path_dependent_evaluators(
      std::set<Evaluator *> & /*evals*/) override {}

  static void add_options_to_parser(options::OptionParser &parser);

  virtual EvaluationResult compute_result(
      EvaluationContext &eval_context) override;
};

#endif