#include "regression_successor_generator_factory.h"

#include "../task_utils/successor_generator_factory.h"
#include "regression_successor_generator_internals.h"

#include "../utils/collections.h"
#include "../utils/memory.h"

#include <algorithm>
#include <cassert>

using namespace std;

namespace regression_successor_generator {

struct RegressionPrecondition {
  int var;
  int value;
  int is_negative;

  RegressionPrecondition(const FactPair &fact, bool is_negative)
      : var(fact.var), value(fact.value), is_negative(is_negative ? 1 : 0) {}
};

struct RegressionOperatorRange {
  int begin;
  int end;

  RegressionOperatorRange(int begin, int end) : begin(begin), end(end) {}

  bool empty() const { return begin == end; }

  int span() const { return end - begin; }
};

class RegressionOperatorInfo {
  OperatorID op;
  vector<RegressionPrecondition> precondition;

 public:
  RegressionOperatorInfo(OperatorID op,
                         vector<RegressionPrecondition> precondition)
      : op(op), precondition(move(precondition)) {}

  bool operator<(const RegressionOperatorInfo &other) const {
    return precondition < other.precondition;
  }

  OperatorID get_op() const { return op; }

  // Returns -1 as a past-the-end sentinel.
  FactPair get_fact(int depth) const {
    if (depth == static_cast<int>(precondition.size())) {
      return FactPair(-1, -1);
    } else {
      return FactPair(precondition[depth].var, precondition[depth].value);
    }
  }

  bool is_negative(int depth) const { precondition[depth].is_negative == 1; }
};

class RegressionOperatorGrouper {
  const vector<RegressionOperatorInfo> &operator_infos;
  const int depth;
  RegressionOperatorRange range;

  const RegressionOperatorInfo &get_current_op_info() const {
    assert(!range.empty());
    return operator_infos[range.begin];
  }

  FactPair get_current_fact() const {
    const RegressionOperatorInfo &op_info = get_current_op_info();
    return op_info.get_fact(depth);
  }

  bool get_current_is_negative() const {
    const RegressionOperatorInfo &op_info = get_current_op_info();
    return op_info.is_negative(depth);
  }

 public:
  explicit RegressionOperatorGrouper(
      const vector<RegressionOperatorInfo> &operator_infos, int depth,
      RegressionOperatorRange range)
      : operator_infos(operator_infos), depth(depth), range(range) {}

  bool done() const { return range.empty(); }

  pair<FactPair, RegressionOperatorRange> next_by_fact() {
    assert(!range.empty());
    FactPair key = get_current_fact();
    int group_begin = range.begin;
    do {
      ++range.begin;
    } while (!range.empty() && get_current_fact() == key);
    RegressionOperatorRange group_range(group_begin, range.begin);
    return make_pair(key, group_range);
  }

  pair<bool, RegressionOperatorRange> next_by_is_negative() {
    assert(!range.empty());
    bool key = get_current_is_negative();
    int group_begin = range.begin;
    do {
      ++range.begin;
    } while (!range.empty() && get_current_is_negative() == key);
    RegressionOperatorRange group_range(group_begin, range.begin);
    return make_pair(key, group_range);
  }
};

RegressionSuccessorGeneratorFactory::RegressionSuccessorGeneratorFactory(
    shared_ptr<const tasks::RegressionTask> task)
    : task(task) {}

RegressionSuccessorGeneratorFactory::~RegressionSuccessorGeneratorFactory() =
    default;

GeneratorPtr RegressionSuccessorGeneratorFactory::construct_fork(
    vector<GeneratorPtr> nodes) const {
  int size = nodes.size();
  if (size == 1) {
    return move(nodes.at(0));
  } else if (size == 2) {
    return utils::make_unique_ptr<successor_generator::GeneratorForkBinary>(
        move(nodes.at(0)), move(nodes.at(1)));
  } else {
    /* This general case includes the case size == 0, which can
       (only) happen for the root for tasks with no operators. */
    return utils::make_unique_ptr<successor_generator::GeneratorForkMulti>(
        move(nodes));
  }
}

GeneratorPtr RegressionSuccessorGeneratorFactory::construct_recursive(
    int depth, RegressionOperatorRange range) const {
  vector<GeneratorPtr> nodes;
  RegressionOperatorGrouper grouper(operator_infos, depth, range);
  while (!grouper.done()) {
    auto fact_group = grouper.next_by_fact();
    FactPair fact = fact_group.first;
    RegressionOperatorRange fact_range = fact_group.second;

    if (fact.var == -1) {
      nodes.push_back(construct_leaf(fact_range));
    } else {
      GeneratorPtr true_generator = nullptr;
      GeneratorPtr false_generator = nullptr;

      while (!grouper.done()) {
        auto is_negative_group = grouper.next_by_is_negative();
        bool is_negative = is_negative_group.first;
        RegressionOperatorRange is_negative_range = is_negative_group.second;

        if (is_negative) {
          false_generator = construct_recursive(depth + 1, is_negative_range);
        } else {
          true_generator = construct_recursive(depth + 1, is_negative_range);
        }
      }

      nodes.push_back(utils::make_unique_ptr<GeneratorSwitchFact>(
          fact, move(true_generator), move(false_generator)));
    }
  }
  return construct_fork(move(nodes));
}

vector<RegressionPrecondition>
RegressionSuccessorGeneratorFactory::build_sorted_precondition(int op_index) {
  vector<RegressionPrecondition> precond;

  int num_preconditions = task->get_num_operator_preconditions(op_index, false);
  precond.reserve(num_preconditions);

  for (int i = 0; i < num_preconditions; ++i) {
    auto fact = task->get_operator_precondition(op_index, i, false);
    int v = task->is_negative_precondition(op_index, i, false) ? 1 : 0;
    precond.emplace_back(RegressionPrecondition(fact, v));
  }

  sort(precond.begin(), precond.end());

  return precond;
}

GeneratorPtr RegressionSuccessorGeneratorFactory::create() {
  operator_infos.reserve(task->get_num_operators());

  for (int i = 0; i < task->get_num_operators(); ++i) {
    auto precond = build_sorted_precondition(i);
    operator_infos.emplace_back(OperatorID(i), precond);
  }

  stable_sort(operator_infos.begin(), operator_infos.end());

  RegressionOperatorRange full_range(0, operator_infos.size());
  GeneratorPtr root = construct_recursive(0, full_range);
  operator_infos.clear();
  return root;
}
}  // namespace regression_successor_generator
