#include "regression_successor_generator.h"

#include "../global_state.h"
#include "regression_successor_generator_factory.h"

using namespace std;

namespace regression_successor_generator {
RegressionSuccessorGenerator::RegressionSuccessorGenerator(
    shared_ptr<const tasks::RegressionTask> task)
    : root(RegressionSuccessorGeneratorFactory(task).create()) {}

RegressionSuccessorGenerator::~RegressionSuccessorGenerator() = default;

void RegressionSuccessorGenerator::generate_applicable_ops(
    const State &state, vector<OperatorID> &applicable_ops) const {
  root->generate_applicable_ops(state, applicable_ops);
}

void RegressionSuccessorGenerator::generate_applicable_ops(
    const GlobalState &state, vector<OperatorID> &applicable_ops) const {
  root->generate_applicable_ops(state, applicable_ops);
}

}  // namespace regression_successor_generator
