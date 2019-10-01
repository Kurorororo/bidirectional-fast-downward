#ifndef FRONT_TO_FRONT_G_EVALUATOR_H
#define FRONT_TO_FRONT_G_EVALUATOR_H

#include "front_to_front_heuristic.h"

namespace options {
class OptionParser;
class Options;
}  // namespace options

namespace front_to_front_g_evaluator {
class FrontToFrontGEvaluator : public FrontToFrontHeuristic {
 protected:
  virtual int compute_heuristic(const GlobalState &state) override {
    return NO_VALUE;
  }

 public:
  FrontToFrontGEvaluator(const options::Options &opts)
      : FrontToFrontHeuristic(opts) {}
  virtual ~FrontToFrontGEvaluator() override = default;

  virtual void set_goal(const GlobalState &state) override {}

  virtual EvaluationResult compute_result(
      EvaluationContext &eval_context) override;

  virtual void get_path_dependent_evaluators(std::set<Evaluator *> &) override {
  }
};
}  // namespace front_to_front_g_evaluator

#endif
