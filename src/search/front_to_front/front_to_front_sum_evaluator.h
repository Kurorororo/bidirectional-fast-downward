#ifndef FRONT_TO_FRONT_SUM_EVALUATOR_H
#define FRONT_TO_FRONT_SUM_EVALUATOR_H

#include "front_to_front_combining_evaluator.h"

#include <memory>
#include <vector>

namespace options {
class Options;
}

namespace front_to_front_sum_evaluator {
class FrontToFrontSumEvaluator : public front_to_front_combining_evaluator::
                                     FrontToFrontCombiningEvaluator {
 protected:
  virtual int combine_values(const std::vector<int> &values) override;

 public:
  explicit FrontToFrontSumEvaluator(const options::Options &opts);
  explicit FrontToFrontSumEvaluator(
      const std::vector<std::shared_ptr<FrontToFrontHeuristic>> &evals);
  virtual ~FrontToFrontSumEvaluator() override;
};
}  // namespace front_to_front_sum_evaluator

#endif
