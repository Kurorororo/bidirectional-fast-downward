#ifndef REGRESSION_TASK_H
#define REGRESSION_TASK_H

#include <memory>
#include <stack>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "../abstract_task.h"
#include "../tasks/root_task.h"

using namespace std;

namespace options {
class OptionParser;
}

namespace tasks {

class RegressionTask : public AbstractTask {
  vector<int> initial_state_values;
  vector<FactPair> goals;
  vector<ExplicitOperator> operators;
  vector<vector<bool>> is_negative_precondition;
  vector<vector<vector<FactPair>>> fact_to_mutexes;

 protected:
  const std::shared_ptr<AbstractTask> parent;

  void init_mutex();
  void reverse_operators();

 public:
  RegressionTask(const std::shared_ptr<AbstractTask> &parent);
  virtual ~RegressionTask() override = default;

  bool is_negative_precondition(int op_index, int fact_index,
                                bool is_axiom) const;

  virtual int get_num_variables() const override;
  virtual std::string get_variable_name(int var) const override;
  virtual int get_variable_domain_size(int var) const override;
  virtual int get_variable_axiom_layer(int var) const override;
  virtual int get_variable_default_axiom_value(int var) const override;
  virtual std::string get_fact_name(const FactPair &fact) const override;
  virtual bool are_facts_mutex(const FactPair &fact1,
                               const FactPair &fact2) const override;

  virtual int get_operator_cost(int index, bool is_axiom) const override;
  virtual std::string get_operator_name(int index,
                                        bool is_axiom) const override;
  virtual int get_num_operators() const override;
  virtual int get_num_operator_preconditions(int index,
                                             bool is_axiom) const override;
  virtual FactPair get_operator_precondition(int op_index, int fact_index,
                                             bool is_axiom) const override;
  virtual int get_num_operator_effects(int op_index,
                                       bool is_axiom) const override;
  virtual int get_num_operator_effect_conditions(int op_index, int eff_index,
                                                 bool is_axiom) const override;
  virtual FactPair get_operator_effect_condition(int op_index, int eff_index,
                                                 int cond_index,
                                                 bool is_axiom) const override;
  virtual FactPair get_operator_effect(int op_index, int eff_index,
                                       bool is_axiom) const override;
  virtual int convert_operator_index(
      int index, const AbstractTask *ancestor_task) const final override;
  virtual int convert_operator_index_to_parent(int index) const {
    return index;
  }

  virtual int get_num_axioms() const override;

  virtual int get_num_goals() const override;
  virtual FactPair get_goal_fact(int index) const override;

  virtual std::vector<int> get_initial_state_values() const override;

  virtual void convert_state_values(
      std::vector<int> &values,
      const AbstractTask *ancestor_task) const final override;
  virtual void convert_state_values_from_parent(std::vector<int> &) const {}

  static std::shared_ptr<AbstractTask> get_regression_task() {
    static std::shared_ptr<AbstractTask> task =
        std::make_shared<RegressionTask>(g_root_task);

    return task;
  }
};

}  // namespace tasks

#endif