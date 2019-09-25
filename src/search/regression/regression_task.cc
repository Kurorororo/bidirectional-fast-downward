#include "regression_task.h"

#include "../landmarks/exploration.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../task_proxy.h"
#include "../utils/rng.h"
#include "../utils/system.h"

#include <algorithm>
#include <iostream>
#include <memory>

using namespace std;

namespace tasks {

void RegressionTask::init_mutex() {
  fact_to_mutexes.resize(get_num_variables(), vector<vector<FactPair>>());

  for (int var1 = 0; var1 < get_num_variables(); ++var1)
    fact_to_mutexes[var1].resize(get_variable_domain_size(var1),
                                 vector<FactPair>());

  for (int var1 = 0; var1 < get_num_variables(); ++var1) {
    for (int value1 = 0; value1 < get_variable_domain_size(var1) - 1;
         ++value1) {
      FactPair fact1(var1, value1);

      for (int var2 = var1 + 1; var2 < get_num_variables(); ++var2) {
        for (int value2 = 0; value2 < get_variable_domain_size(var2) - 1;
             ++value2) {
          FactPair fact2(var2, value2);

          if (are_facts_mutex(fact1, fact2)) {
            fact_to_mutexes[var1][value1].push_back(fact2);
            fact_to_mutexes[var2][value2].push_back(fact1);
          }
        }
      }
    }
  }
}

void RegressionTask::reverse_operators() {
  cout << "inversing operators" << endl;

  if (parent->get_num_axioms() > 0)
    throw runtime_error("Axiom is not supported.");

  vector<int> effect_var_values(get_num_variables());
  vector<int> precondition_var_values(get_num_variables());

  for (int i = 0; i < parent->get_num_operators(); ++i) {
    fill(effect_var_values.begin(), effect_var_values.end(), -1);
    fill(precondition_var_values.begin(), precondition_var_values.end(), -1);

    vector<FactPair> new_preconditions;
    vector<FactPair> new_negative_preconditons;
    vector<ExplicitEffect> new_effects;

    for (int j = 0; j < parent->get_num_operator_effects(i, false); ++j) {
      if (parent->get_num_operator_effect_conditions(i, j, false) > 0)
        throw runtime_error("Conditional effects are not supported.");

      FactPair fact = parent->get_operator_effect(i, j, false);
      effect_var_values[fact.var] = fact.value;
      new_preconditions.push_back(fact);

      for (auto f : fact_to_mutexes[fact.var][fact.value])
        new_negative_preconditons.push_back(f);
    }

    for (int j = 0; j < parent->get_num_operator_preconditions(i, false); ++j) {
      FactPair fact = parent->get_operator_precondition(i, j, false);
      precondition_var_values[fact.var] = fact.value;

      for (auto f : fact_to_mutexes[fact.var][fact.value])
        if (effect_var_values[f.var] != f.value)
          new_negative_preconditons.push_back(f);

      if (effect_var_values[fact.var] == -1 ||
          effect_var_values[fact.var] == fact.value)
        new_preconditions.push_back(fact);

      new_effects.emplace_back(
          ExplicitEffect(fact.var, fact.value, move(vector<FactPair>())));
    }

    for (int var = 0, n = effect_var_values.size(); var < n; ++var) {
      if (effect_var_values[var] != -1 && precondition_var_values[var] == -1)
        new_effects.emplace_back(ExplicitEffect(
            var, get_variable_domain_size(var) - 1, move(vector<FactPair>())));
    }

    vector<bool> is_negative(new_preconditions.size(), false);

    sort(new_negative_preconditons.begin(), new_negative_preconditons.end());
    auto result = unique(new_negative_preconditons.begin(),
                         new_negative_preconditons.end());
    new_negative_preconditons.erase(result, new_negative_preconditons.end());
    new_preconditions.insert(new_preconditions.end(),
                             new_negative_preconditons.begin(),
                             new_negative_preconditons.end());

    is_negative.resize(new_preconditions.size(), true);
    is_negative_precondition_vector.emplace_back(move(is_negative));

    int cost = parent->get_operator_cost(i, false);
    string name = parent->get_operator_name(i, false);
    operators.emplace_back(ExplicitOperator(
        move(new_preconditions), move(new_effects), cost, name, false));
  }
}

RegressionTask::RegressionTask(const shared_ptr<AbstractTask> &parent)
    : parent(parent) {
  init_mutex();
  reverse_operators();
}

vector<int> RegressionTask::get_goal_state_values() const {
  vector<int> values(get_num_variables());

  for (int var = 0; var < get_num_variables(); ++var)
    values[var] = get_variable_domain_size(var) - 1;

  for (int i = 0; i < get_num_goals(); ++i) {
    FactPair fact = get_goal_fact(i);
    values[fact.var] = fact.value;
  }

  vector<unordered_set<int>> ranges(get_num_variables(), unordered_set<int>());

  for (int var = 0; var < get_num_variables(); ++var)
    for (int value = 0; value < get_variable_domain_size(var) - 1; ++value)
      ranges[var].insert(value);

  for (int var = 0; var < get_num_variables(); ++var) {
    int value = values[var];

    if (value == get_variable_domain_size(var) - 1) continue;

    for (auto fact : fact_to_mutexes[var][value]) {
      auto result = ranges[fact.var].find(fact.value);
      if (result != ranges[fact.var].end()) ranges[fact.var].erase(result);
    }
  }

  for (int var = 0; var < get_num_variables(); ++var) {
    int value = values[var];

    if (value != get_variable_domain_size(var) - 1) continue;

    if (ranges[var].size() == 1) values[var] = *ranges[var].begin();
  }

  return values;
}

bool RegressionTask::is_negative_precondition(int op_index, int fact_index,
                                              bool is_axiom) const {
  return is_negative_precondition_vector[op_index][fact_index];
}

int RegressionTask::get_num_variables() const {
  return parent->get_num_variables();
}

string RegressionTask::get_variable_name(int var) const {
  return parent->get_variable_name(var);
}

int RegressionTask::get_variable_domain_size(int var) const {
  return parent->get_variable_domain_size(var);
}

int RegressionTask::get_variable_axiom_layer(int var) const {
  return parent->get_variable_axiom_layer(var);
}

int RegressionTask::get_variable_default_axiom_value(int var) const {
  return parent->get_variable_default_axiom_value(var);
}

string RegressionTask::get_fact_name(const FactPair &fact) const {
  return parent->get_fact_name(fact);
}

bool RegressionTask::are_facts_mutex(const FactPair &fact1,
                                     const FactPair &fact2) const {
  return parent->are_facts_mutex(fact1, fact2);
}

int RegressionTask::get_operator_cost(int index, bool is_axiom) const {
  return operators[index].cost;
}

string RegressionTask::get_operator_name(int index, bool is_axiom) const {
  return operators[index].name;
}

int RegressionTask::get_num_operators() const { return operators.size(); }

int RegressionTask::get_num_operator_preconditions(int index,
                                                   bool is_axiom) const {
  return operators[index].preconditions.size();
}

FactPair RegressionTask::get_operator_precondition(int op_index, int fact_index,
                                                   bool is_axiom) const {
  return operators[op_index].preconditions[fact_index];
}

int RegressionTask::get_num_operator_effects(int op_index,
                                             bool is_axiom) const {
  return operators[op_index].effects.size();
}

int RegressionTask::get_num_operator_effect_conditions(int op_index,
                                                       int eff_index,
                                                       bool is_axiom) const {
  return 0;
}

FactPair RegressionTask::get_operator_effect_condition(int op_index,
                                                       int eff_index,
                                                       int cond_index,
                                                       bool is_axiom) const {
  return parent->get_operator_effect_condition(op_index, eff_index, cond_index,
                                               is_axiom);
}

FactPair RegressionTask::get_operator_effect(int op_index, int eff_index,
                                             bool is_axiom) const {
  return operators[op_index].effects[eff_index].fact;
}

int RegressionTask::convert_operator_index(
    int index, const AbstractTask *ancestor_task) const {
  if (ancestor_task == this) {
    return index;
  }
  return parent->convert_operator_index(index, ancestor_task);
}

int RegressionTask::get_num_axioms() const { return 0; }

int RegressionTask::get_num_goals() const { return parent->get_num_goals(); }

FactPair RegressionTask::get_goal_fact(int index) const {
  return parent->get_goal_fact(index);
}

vector<int> RegressionTask::get_initial_state_values() const {
  return parent->get_initial_state_values();
}

void RegressionTask::convert_state_values(
    vector<int> &values, const AbstractTask *ancestor_task) const {
  if (this == ancestor_task) {
    return;
  }
  parent->convert_state_values(values, ancestor_task);
  convert_state_values_from_parent(values);
}

static shared_ptr<AbstractTask> _parse(OptionParser &parser) {
  Options opts = parser.parse();

  if (parser.dry_run())
    return nullptr;
  else
    return RegressionTask::get_regression_task();
}

static Plugin<AbstractTask> _plugin("regression", _parse);
}  // namespace tasks
