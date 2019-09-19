#include "inverse_task.h"

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

void InverseTask::init_name_to_fact() {
  for (int var = 0, n = get_num_variables(); var < n; ++var) {
    for (int val = 0, m = get_variable_domain_size(var); val < m; ++val) {
      FactPair fact(var, val);
      string name = get_fact_name(fact);

      if (name == "<none of those>")
        none_of_those_value[var] = val;
      else
        name_to_fact[name] = make_pair(fact.var, fact.value);
    }
  }
}

FactPair InverseTask::get_negation(const FactPair &fact) const {
  auto name = get_fact_name(fact);

  if (name.substr(0, 5) == "Atom ") {
    auto negated_name = "Negated" + name;
    auto result = name_to_fact.find(negated_name);

    if (result != name_to_fact.end())
      return FactPair(result->second.first, result->second.second);
    else if (none_of_those_value[fact.var])
      return FactPair(fact.var, none_of_those_value[fact.var]);
  } else if (name.substr(0, 7) == "Negated") {
    auto negated_name = name.substr(7, name.size());
    auto result = name_to_fact.find(negated_name);

    if (result != name_to_fact.end())
      return FactPair(result->second.first, result->second.second);
    else if (none_of_those_value[fact.var])
      return FactPair(fact.var, none_of_those_value[fact.var]);
  }

  return FactPair(-1, -1);
}

void InverseTask::reverse_operators() {
  cout << "inversing operators" << endl;
  init_name_to_fact();

  if (parent->get_num_axioms() > 0)
    throw runtime_error("Axiom is not supported.");

  vector<int> precondition_var_values(get_num_variables());
  unordered_set<int> new_precondition_vars;
  unordered_set<int> new_effect_vars;

  for (int i = 0; i < parent->get_num_operators(); ++i) {
    fill(precondition_var_values.begin(), precondition_var_values.end(), -1);
    new_precondition_vars.clear();
    new_effect_vars.clear();

    for (int j = 0; j < parent->get_num_operator_preconditions(i, false); ++j) {
      FactPair fact = parent->get_operator_precondition(i, j, false);
      precondition_var_values[fact.var] = fact.value;
    }

    vector<FactPair> new_preconditions;
    vector<ExplicitEffect> new_effects;

    for (int j = 0; j < parent->get_num_operator_effects(i, false); ++j) {
      if (parent->get_num_operator_effect_conditions(i, j, false) > 0)
        throw runtime_error("Conditional effects are not supported.");

      FactPair fact = parent->get_operator_effect(i, j, false);

      if (new_precondition_vars.find(fact.var) == new_precondition_vars.end()) {
        new_preconditions.push_back(fact);
        new_precondition_vars.insert(fact.var);

        if (precondition_var_values[fact.var] != -1 &&
            precondition_var_values[fact.var] != fact.value) {
          new_effects.emplace_back(
              ExplicitEffect(fact.var, precondition_var_values[fact.var],
                             move(vector<FactPair>())));
          new_effect_vars.insert(fact.var);
        }
      }
    }

    for (int j = 0; j < parent->get_num_operator_effects(i, false); ++j) {
      FactPair fact = parent->get_operator_effect(i, j, false);

      if (new_effect_vars.find(fact.var) != new_effect_vars.end()) continue;

      FactPair negated_fact = get_negation(fact);

      if (negated_fact.var != -1 && negated_fact.value != -1) {
        if (new_effect_vars.find(negated_fact.var) == new_effect_vars.end()) {
          new_effects.emplace_back(ExplicitEffect(
              negated_fact.var, negated_fact.value, move(vector<FactPair>())));
          new_effect_vars.insert(negated_fact.var);
        }

        if (precondition_var_values[negated_fact.var] == negated_fact.value)
          new_precondition_vars.insert(negated_fact.var);
      }
    }

    for (int var = 0; var < get_num_variables(); ++var) {
      if (precondition_var_values[var] != -1 &&
          new_precondition_vars.find(var) == new_precondition_vars.end()) {
        new_preconditions.emplace_back(
            FactPair(var, precondition_var_values[var]));
      }
    }

    int cost = parent->get_operator_cost(i, false);
    string name = parent->get_operator_name(i, false);

    operators.emplace_back(ExplicitOperator(
        move(new_preconditions), move(new_effects), cost, name, false));
  }
}

void InverseTask::propagate_mutex(int var, int value,
                                  vector<vector<int>> &ranges) {
  for (auto f : fact_to_mutexes[var][value]) {
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

  for (int i = 0; i < parent->get_num_goals(); ++i) {
    FactPair fact1 = parent->get_goal_fact(i);
    propagate_mutex(fact1.var, fact1.value, ranges);
  }

  for (int var = 0; var < get_num_variables(); ++var) {
    if (ranges[var].size() == 1) {
      initial_state_values[var] = ranges[var][0];
      ranges[var].clear();
    } else if (!ranges[var].empty()) {
      to_be_filled.push_back(var);
    }
  }

  if (value_ordering == Ordering::MUTEX) {
    for (auto var : to_be_filled) {
      auto compare = [this, var](int val1, int val2) {
        return fact_to_mutexes[var][val1].size() >
               fact_to_mutexes[var][val2].size();
      };

      sort(ranges[var].begin(), ranges[var].end(), compare);
    }
  }
}

int InverseTask::find_next_variable(int var, const vector<int> &values,
                                    const vector<vector<int>> &ranges) {
  int next = -1;

  if (variable_ordering == Ordering::DEFAULT) {
    next = var + 1;

    while (next < get_num_variables() &&
           (values[next] != -1 || ranges[next].empty()))
      ++next;

    if (next == get_num_variables()) next = -1;
  }

  if (variable_ordering == Ordering::REVERSE) {
    next = get_num_variables() - 1;

    while (next > 0 && (values[next] != -1 || ranges[next].empty())) --next;
  }

  if (variable_ordering == RANDOM) {
    int counter = 1;

    for (auto v : to_be_filled) {
      if (values[v] != -1) continue;

      if (ranges[v].empty()) return v;

      if (rng->operator()(counter) == 0) next = v;

      ++counter;
    }
  }

  if (variable_ordering == MUTEX) {
    int min = -1;

    for (auto v : to_be_filled) {
      if (values[v] != -1) continue;

      if (ranges[v].empty()) return v;

      int sum = 0;

      for (auto u : fact_to_mutexes[v]) sum += u.size();

      if (next == -1 || sum < min) {
        next = v;
        min = sum;
      }
    }
  }

  if (variable_ordering == Ordering::RANGE) {
    int min = -1;

    for (auto v : to_be_filled) {
      if (values[v] != -1) continue;

      if (ranges[v].empty()) return v;

      if (next == -1 || ranges[v].size() < min) {
        next = v;
        min = ranges[v].size();
      }
    }
  }

  return next;
}

bool InverseTask::informed_backtracking(const vector<vector<int>> &ranges,
                                        int var) {
  if (var == -1) {
    TaskProxy proxy(*this);
    landmarks::Exploration exploration(proxy);

    return exploration.compute_reachability();
  }

  if (ranges[var].empty()) return false;

  int start = 0;

  if (value_ordering == Ordering::RANDOM)
    start = rng->operator()(ranges[var].size());

  for (int i = 0, n = ranges[var].size(); i < n; ++i) {
    int j = value_ordering == Ordering::REVERSE ? n - i - 1 : i;
    int index = (start + j) % ranges[var].size();
    int value = ranges[var][index];
    initial_state_values[var] = value;
    vector<vector<int>> child_ranges(ranges);
    propagate_mutex(var, value, child_ranges);
    int next_var = find_next_variable(var, initial_state_values, child_ranges);

    if (informed_backtracking(child_ranges, next_var)) return true;
  }

  initial_state_values[var] = -1;

  return false;
}

bool InverseTask::informed_dfs() {
  stack<shared_ptr<DFSNode>> open;
  int var = find_next_variable(-1, initial_state_values, ranges);
  open.push(make_shared<DFSNode>(var, initial_state_values, ranges));

  while (!open.empty()) {
    auto top = open.top();
    open.pop();
    int var = top->var;

    if (var == -1) {
      initial_state_values.swap(top->values);
      TaskProxy proxy(*this);
      landmarks::Exploration exploration(proxy);

      if (exploration.compute_reachability()) {
        return true;
      } else {
        initial_state_values.swap(top->values);
        continue;
      }
    }

    if (top->ranges[var].empty()) return false;

    vector<shared_ptr<DFSNode>> tmp;

    for (int i = 0, n = top->ranges[var].size(); i < n; ++i) {
      auto child = make_shared<DFSNode>(var, top->values, top->ranges);
      int value = ranges[var][i];
      child->values[var] = value;
      propagate_mutex(var, value, child->ranges);
      child->var = find_next_variable(var, child->values, child->ranges);
      tmp.push_back(child);
    }

    // auto compare = [this, var](shared_ptr<const DFSNode> node1,
    //                           shared_ptr<const DFSNode> node2) {
    //  int sum1 = 0;

    //  for (int v = 0; v < get_num_variables(); ++v)
    //    if (node1->values[v] == -1) sum1 += node1->ranges[v].size();

    //  int sum2 = 0;

    //  for (int v = 0; v < get_num_variables(); ++v)
    //    if (node2->values[v] == -1) sum2 += node2->ranges[v].size();

    //  return sum1 > sum2;
    //};

    // sort(tmp.begin(), tmp.end(), compare);

    for (auto node : tmp) open.push(node);
  }

  return false;
}

void InverseTask::set_initial_state() {
  // cout << "generating an inverse initial state (a goal state)" << endl;
  if (parent->get_num_goals() == get_num_variables()) return;

  bool success = false;

  if (value_ordering == RANGE) {
    success = informed_dfs();
  } else {
    int var = find_next_variable(-1, initial_state_values, ranges);
    success = informed_backtracking(ranges, var);
  }

  if (!success) throw runtime_error("There is no valid goal state!");
}

InverseTask::InverseTask(const shared_ptr<AbstractTask> &parent,
                         Ordering variable_ordering, Ordering value_ordering)
    : initial_state_values(parent->get_num_variables(), -1),
      parent(parent),
      rng(make_shared<utils::RandomNumberGenerator>(1012)),
      ranges(parent->get_num_variables(), vector<int>()),
      none_of_those_value(parent->get_num_variables(), -1),
      variable_ordering(variable_ordering),
      value_ordering(value_ordering) {
  for (int i = 0; i < parent->get_num_goals(); ++i) {
    auto fact = parent->get_goal_fact(i);
    initial_state_values[fact.var] = fact.value;
  }

  auto parent_initial_state_values = parent->get_initial_state_values();

  for (int i = 0; i < parent->get_num_variables(); ++i)
    goals.push_back(FactPair(i, parent_initial_state_values[i]));

  reverse_operators();

  if (parent->get_num_goals() < parent->get_num_variables()) {
    cout << "Warning: there are multiple goal states." << endl;
    cout << "Generating a goal state..." << endl;

    init_mutex();
    init_ranges();
    set_initial_state();
  }
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
  // vector<string> variable_orderings;
  // vector<string> variable_orderings_doc;
  // variable_orderings.push_back("DEFAULT");
  // variable_orderings_doc.push_back("fast-downward order");
  // variable_orderings.push_back("RANDOM");
  // variable_orderings_doc.push_back("random choice");
  // variable_orderings.push_back("MUTEX");
  // variable_orderings_doc.push_back(
  //    "select one with the highest number of mutex groups");
  // variable_orderings.push_back("RANGE");
  // variable_orderings_doc.push_back(
  //    "select one with the lowest number of value options");
  // parser.add_enum_option("var_order", variable_orderings,
  //                       "order to select variables in the goal generation",
  //                       "DEFAULT", variable_orderings_doc);

  // vector<string> value_orderings;
  // vector<string> value_orderings_doc;
  // value_orderings.push_back("DEFAULT");
  // value_orderings_doc.push_back("fast-downward order");
  // value_orderings.push_back("RANDOM");
  // value_orderings_doc.push_back("random choice");
  // value_orderings.push_back("MUTEX");
  // value_orderings_doc.push_back(
  //    "select one with the highest number of mutex groups");
  // value_orderings.push_back("RANGE");
  // value_orderings_doc.push_back(
  //    "select one with the lowest number of value options");
  // parser.add_enum_option("val_order", value_orderings,
  //                       "order to select values in the goal generation",
  //                       "DEFAULT", value_orderings_doc);

  Options opts = parser.parse();
  // InverseTask::Ordering variable_ordering(opts.get_enum("var_order"));
  // InverseTask::Ordering value_ordering(opts.get_enum("val_order"));

  if (parser.dry_run())
    return nullptr;
  else
    return InverseTask::get_inverse_task();
}

static Plugin<AbstractTask> _plugin("inverse", _parse);
}  // namespace tasks
