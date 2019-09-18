#include "inverse_task.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "../utils/rng.h"
#include "../utils/system.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <utility>

using namespace std;

namespace tasks {

void InverseTask::reverse_operators() {
  cout << "inversing operators" << endl;

  if (parent->get_num_axioms() > 0)
    throw runtime_error("Axiom is not supported.");

  unordered_map<string, pair<int, int>> name_to_fact;

  for (int var = 0, n = get_num_variables(); var < n; ++var) {
    for (int val = 0, m = get_variable_domain_size(var); val < m; ++val) {
      FactPair fact(var, val);
      name_to_fact[get_fact_name(fact)] = make_pair(fact.var, fact.value);
    }
  }

  unordered_set<string> old_positive_precondition;
  unordered_set<string> old_negative_precondition;
  unordered_set<string> old_add_effects;
  unordered_set<string> old_del_effects;

  unordered_set<string> new_positive_precondition;
  unordered_set<string> new_negative_precondition;

  vector<int> precondition_var_values(get_num_variables());

  for (int i = 0; i < parent->get_num_operators(); ++i) {
    old_positive_precondition.clear();
    old_negative_precondition.clear();
    old_add_effects.clear();
    old_del_effects.clear();

    new_positive_precondition.clear();
    new_negative_precondition.clear();

    fill(precondition_var_values.begin(), precondition_var_values.end(), -1);

    for (int j = 0; j < parent->get_num_operator_preconditions(i, false); ++j) {
      FactPair fact = parent->get_operator_precondition(i, j, false);
      precondition_var_values[fact.var] = fact.value;
      string name = get_fact_name(fact);

      if (name.substr(0, 5) == "Atom ")
        old_positive_precondition.insert(name);
      else if (name.substr(0, 7) == "Negated")
        old_negative_precondition.insert(name.substr(8, name.size()));
    }

    for (int j = 0; j < parent->get_num_operator_effects(i, false); ++j) {
      if (parent->get_num_operator_effect_conditions(i, j, false) > 0)
        throw runtime_error("Conditional effects are not supported.");

      FactPair fact = parent->get_operator_effect(i, j, false);

      if (precondition_var_values[fact.var] != -1 &&
          precondition_var_values[fact.var] != fact.value) {
        FactPair pre_fact(fact.var, precondition_var_values[fact.var]);
        string pre_name = get_fact_name(pre_fact);

        if (pre_name.substr(0, 5) == "Atom ")
          old_del_effects.insert(pre_name);
        else if (pre_name.substr(0, 7) == "Negated")
          old_add_effects.insert(pre_name.substr(8, pre_name.size()));
      }

      string name = get_fact_name(fact);

      if (name.substr(0, 5) == "Atom ")
        old_add_effects.insert(name);
      else if (name.substr(0, 7) == "Negated")
        old_del_effects.insert(name.substr(8, name.size()));
    }

    for (auto name : old_positive_precondition)
      if (old_del_effects.find(name) == old_del_effects.end())
        new_positive_precondition.insert(name);

    for (auto name : old_add_effects) new_positive_precondition.insert(name);

    for (auto name : old_negative_precondition)
      if (old_add_effects.find(name) == old_add_effects.end())
        new_negative_precondition.insert(name);

    for (auto name : old_del_effects) new_negative_precondition.insert(name);

    vector<FactPair> new_preconditions;

    for (auto name : new_positive_precondition) {
      auto result = name_to_fact.find(name);

      if (result != name_to_fact.end()) {
        int var = result->second.first;
        int value = result->second.second;
        new_preconditions.push_back(FactPair(var, value));
      }
    }

    for (auto name : new_negative_precondition) {
      auto negated_name = "Negated" + name;
      auto result = name_to_fact.find(negated_name);

      if (result != name_to_fact.end()) {
        int var = result->second.first;
        int value = result->second.second;
        new_preconditions.push_back(FactPair(var, value));
      }
    }

    vector<ExplicitEffect> new_effects;

    for (auto name : old_del_effects) {
      auto result = name_to_fact.find(name);

      if (result != name_to_fact.end()) {
        int var = result->second.first;
        int value = result->second.second;
        new_effects.emplace_back(
            ExplicitEffect(var, value, move(vector<FactPair>())));
      }
    }

    for (auto name : old_add_effects) {
      auto negated_name = "Negated" + name;
      auto result = name_to_fact.find(negated_name);

      if (result != name_to_fact.end()) {
        int var = result->second.first;
        int value = result->second.second;
        new_effects.emplace_back(
            ExplicitEffect(var, value, move(vector<FactPair>())));
      }
    }

    int cost = parent->get_operator_cost(i, false);
    string name = parent->get_operator_name(i, false);

    operators.emplace_back(ExplicitOperator(
        move(new_preconditions), move(new_effects), cost, name, false));
  }
}

void InverseTask::propagate_mutex(const FactPair &fact,
                                  vector<vector<int>> &ranges) {
  for (auto f : fact_to_mutexes[fact.var][fact.value]) {
    ranges[f.var].erase(
        std::remove(ranges[f.var].begin(), ranges[f.var].end(), f.value),
        ranges[f.var].end());
  }
}

void InverseTask::init_mutex() {
  fact_to_mutexes.resize(get_num_variables(), vector<vector<FactPair>>());

  for (int var1 = 0; var1 < get_num_variables(); ++var1)
    fact_to_mutexes[var1].resize(get_variable_domain_size(var1),
                                 vector<FactPair>());

  for (int var1 = 0; var1 < get_num_variables(); ++var1) {
    for (int value1 = 0; value1 < get_variable_domain_size(var1); ++value1) {
      FactPair fact1(var1, value1);

      for (int var2 = var1 + 1; var2 < get_num_variables(); ++var2) {
        for (int value2 = 0; value2 < get_variable_domain_size(var2);
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

void InverseTask::init_ranges() {
  for (int var = 0; var < get_num_variables(); ++var)
    for (int val = 0, m = get_variable_domain_size(var); val < m; ++val)
      ranges[var].push_back(val);

  for (int i = 0; i < parent->get_num_goals(); ++i) {
    FactPair fact1 = parent->get_goal_fact(i);
    ranges[fact1.var].clear();
  }

  for (int i = 0; i < get_num_variables(); ++i)
    if (!ranges[i].empty()) to_be_filled.push_back(i);

  for (int i = 0; i < parent->get_num_goals(); ++i) {
    FactPair fact1 = parent->get_goal_fact(i);
    propagate_mutex(fact1, ranges);
  }
}

int InverseTask::find_next_variable(const vector<vector<int>> &ranges) {
  int min = -1;
  int arg_min = -1;

  for (auto v : to_be_filled) {
    if (initial_state_values[v] != -1) continue;

    if (ranges[v].empty()) return v;

    if (arg_min == -1 || ranges[v].size() < min) {
      arg_min = v;
      min = ranges[v].size();
    }
  }

  return arg_min;
}

bool InverseTask::informed_backtracking(const vector<vector<int>> &ranges,
                                        int var, vector<int> &values) {
  if (var == -1) return true;

  if (ranges[var].empty()) return false;

  int start = rng->operator()(ranges[var].size());

  for (int i = 0, n = ranges[var].size(); i < n; ++i) {
    int index = (start + i) % ranges[var].size();
    int value = ranges[var][index];
    values[var] = value;
    vector<vector<int>> child_ranges(ranges);
    propagate_mutex(FactPair(var, value), child_ranges);
    int next_var = find_next_variable(child_ranges);

    if (informed_backtracking(child_ranges, next_var, values)) return true;
  }

  values[var] = -1;

  return false;
}

void InverseTask::set_initial_state() {
  // cout << "generating an inverse initial state (a goal state)" << endl;
  std::fill(initial_state_values.begin(), initial_state_values.end(), -1);

  for (int i = 0; i < parent->get_num_goals(); ++i) {
    auto fact = parent->get_goal_fact(i);
    initial_state_values[fact.var] = fact.value;
  }

  if (parent->get_num_goals() == static_cast<int>(initial_state_values.size()))
    return;

  int var = find_next_variable(ranges);
  bool success = informed_backtracking(ranges, var, initial_state_values);
  assert(success);
}

InverseTask::InverseTask(const shared_ptr<AbstractTask> &parent)
    : initial_state_values(parent->get_num_variables(), -1),
      parent(parent),
      rng(make_shared<utils::RandomNumberGenerator>(1012)),
      ranges(parent->get_num_variables(), vector<int>()) {
  if (parent->get_num_goals() < parent->get_num_variables()) {
    cout << "Warning: there are multiple goal states" << endl;
    init_mutex();
    init_ranges();
    set_initial_state();
  }

  auto parent_initial_state_values = parent->get_initial_state_values();

  for (int i = 0; i < parent->get_num_variables(); ++i)
    goals.push_back(FactPair(i, parent_initial_state_values[i]));

  reverse_operators();
}

int InverseTask::get_num_variables() const {
  return parent->get_num_variables();
}

string InverseTask::get_variable_name(int var) const {
  return parent->get_variable_name(var);
}

int InverseTask::get_variable_domain_size(int var) const {
  return parent->get_variable_domain_size(var);
}

int InverseTask::get_variable_axiom_layer(int var) const {
  return parent->get_variable_axiom_layer(var);
}

int InverseTask::get_variable_default_axiom_value(int var) const {
  return parent->get_variable_default_axiom_value(var);
}

string InverseTask::get_fact_name(const FactPair &fact) const {
  return parent->get_fact_name(fact);
}

bool InverseTask::are_facts_mutex(const FactPair &fact1,
                                  const FactPair &fact2) const {
  return parent->are_facts_mutex(fact1, fact2);
}

int InverseTask::get_operator_cost(int index, bool is_axiom) const {
  return operators[index].cost;
}

string InverseTask::get_operator_name(int index, bool is_axiom) const {
  return operators[index].name;
}

int InverseTask::get_num_operators() const { return operators.size(); }

int InverseTask::get_num_operator_preconditions(int index,
                                                bool is_axiom) const {
  return operators[index].preconditions.size();
}

FactPair InverseTask::get_operator_precondition(int op_index, int fact_index,
                                                bool is_axiom) const {
  return operators[op_index].preconditions[fact_index];
}

int InverseTask::get_num_operator_effects(int op_index, bool is_axiom) const {
  return operators[op_index].effects.size();
}

int InverseTask::get_num_operator_effect_conditions(int op_index, int eff_index,
                                                    bool is_axiom) const {
  return 0;
}

FactPair InverseTask::get_operator_effect_condition(int op_index, int eff_index,
                                                    int cond_index,
                                                    bool is_axiom) const {
  return parent->get_operator_effect_condition(op_index, eff_index, cond_index,
                                               is_axiom);
}

FactPair InverseTask::get_operator_effect(int op_index, int eff_index,
                                          bool is_axiom) const {
  return operators[op_index].effects[eff_index].fact;
}

int InverseTask::convert_operator_index(
    int index, const AbstractTask *ancestor_task) const {
  if (ancestor_task == this) {
    return index;
  }
  int parent_index = convert_operator_index_to_parent(index);
  return parent->convert_operator_index(parent_index, ancestor_task);
}

int InverseTask::get_num_axioms() const { return 0; }

int InverseTask::get_num_goals() const { return goals.size(); }

FactPair InverseTask::get_goal_fact(int index) const { return goals[index]; }

vector<int> InverseTask::get_initial_state_values() const {
  return initial_state_values;
}

void InverseTask::convert_state_values(
    vector<int> &values, const AbstractTask *ancestor_task) const {
  if (this == ancestor_task) {
    return;
  }
  parent->convert_state_values(values, ancestor_task);
  convert_state_values_from_parent(values);
}

static shared_ptr<AbstractTask> _parse(OptionParser &parser) {
  if (parser.dry_run())
    return nullptr;
  else
    return InverseTask::get_inverse_task();
}

static Plugin<AbstractTask> _plugin("inverse", _parse);
}  // namespace tasks
