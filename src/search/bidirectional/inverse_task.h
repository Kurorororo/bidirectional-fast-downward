#ifndef INVERSE_TASK_H
#define INVERSE_TASK_H

#include <memory>
#include <stack>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "../abstract_task.h"
#include "../tasks/root_task.h"
#include "../utils/rng.h"

using namespace std;

namespace options {
class OptionParser;
}

namespace tasks {

class InverseTask : public AbstractTask {
  struct DFSNode {
    int var;
    vector<int> values;
    vector<vector<int>> ranges;

    DFSNode() {}

    DFSNode(int var, const vector<int> &values,
            const vector<vector<int>> &ranges)
        : var(var), values(values), ranges(ranges) {}
  };

  vector<int> initial_state_values;
  vector<FactPair> goals;
  vector<ExplicitOperator> operators;
  unordered_map<string, pair<int, int>> name_to_fact;
  vector<int> none_of_those_value;
  vector<vector<vector<FactPair>>> fact_to_mutexes;
  vector<vector<int>> ranges;
  vector<int> to_be_filled;
  stack<DFSNode> open;

 protected:
  std::shared_ptr<utils::RandomNumberGenerator> rng;
  const std::shared_ptr<AbstractTask> parent;

  void init_name_to_fact();
  FactPair get_negation(const FactPair &fact) const;
  void reverse_operators();
  void reverse_operators_without_strips_info();
  void propagate_mutex(const FactPair &fact, vector<vector<int>> &ranges);
  void init_mutex();
  void init_ranges();
  int find_next_variable(const vector<int> &values,
                         const vector<vector<int>> &ranges);
  bool informed_backtracking(const vector<vector<int>> &ranges, int var);
  bool informed_dfs();

 public:
  InverseTask(const std::shared_ptr<AbstractTask> &parent);
  virtual ~InverseTask() override = default;

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

  virtual void set_initial_state() override;

  static std::shared_ptr<AbstractTask> get_inverse_task() {
    static std::shared_ptr<AbstractTask> task =
        std::make_shared<InverseTask>(g_root_task);

    return task;
  }
};

}  // namespace tasks

#endif