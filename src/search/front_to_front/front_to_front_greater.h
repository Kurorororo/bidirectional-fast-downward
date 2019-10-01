#ifndef FRONT_TO_FRONT_GREATER_H
#define FRONT_TO_FRONT_GREATER_H

#include "front_to_front_heuristic.h"

#include <memory>

namespace options {
class Options;
}

namespace front_to_front_greater {
class FrontToFrontGreater : public FrontToFrontHeuristic {
  std::shared_ptr<FrontToFrontHeuristic> evaluator;

 protected:
  virtual int compute_heuristic(const GlobalState &state) override {
    return DEAD_END;
  };

 public:
  static const int INFTY;

  explicit FrontToFrontGreater(const options::Options &opts);
  virtual ~FrontToFrontGreater() override;

  virtual void set_goal(const GlobalState &state);

  virtual bool dead_ends_are_reliable() const override;
  virtual void get_path_dependent_evaluators(
      std::set<Evaluator *> &evals) override;
  virtual EvaluationResult compute_result(
      EvaluationContext &eval_context) override;
};
}  // namespace front_to_front_greater

#endif
