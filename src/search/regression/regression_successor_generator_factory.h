#ifndef REGRESSION_SUCCESSOR_GENERATOR_FACTORY_H
#define REGRESSION_SUCCESSOR_GENERATOR_FACTORY_H

#include "../operator_id.h"
#include "../task_utils/successor_generator_factory.h"
#include "../task_utils/successor_generator_internals.h"
#include "regression_task.h"

#include <memory>
#include <utility>
#include <vector>

namespace regression_successor_generator {
using GeneratorPtr = std::unique_ptr<successor_generator::GeneratorBase>;

struct RegressionPrecondition;
struct RegressionOperatorRange;
class RegressionOperatorInfo;

class RegressionSuccessorGeneratorFactory {
  using ValuesAndGenerators = std::vector<std::pair<int, GeneratorPtr>>;

  std::shared_ptr<const tasks::RegressionTask> task;
  std::vector<RegressionOperatorInfo> operator_infos;

  std::vector<RegressionPrecondition> build_sorted_precondition(int op_index);

  GeneratorPtr construct_fork(std::vector<GeneratorPtr> nodes) const;
  GeneratorPtr construct_leaf(RegressionOperatorRange range) const;
  GeneratorPtr construct_recursive(int depth,
                                   RegressionOperatorRange range) const;

 public:
  explicit RegressionSuccessorGeneratorFactory(
      std::shared_ptr<const tasks::RegressionTask> task);
  ~RegressionSuccessorGeneratorFactory();
  GeneratorPtr create();
};

}  // namespace regression_successor_generator

#endif