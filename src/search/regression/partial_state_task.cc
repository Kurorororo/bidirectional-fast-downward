#include "partial_state_task.h"

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

PartialStateTask::PartialStateTask(const shared_ptr<AbstractTask> &parent)
    : parent(parent) {}

vector<int> PartialStateTask::get_goal_state_values() const {
  vector<int> values(get_num_variables());

  for (int var = 0; var < get_num_variables(); ++var)
    values[var] = get_variable_domain_size(var) - 1;

  for (int i = 0; i < get_num_goals(); ++i) {
    FactPair fact = get_goal_fact(i);
    values[fact.var] = fact.value;
  }

  return values;
}

int PartialStateTask::get_num_variables() const {
  return parent->get_num_variables();
}

string PartialStateTask::get_variable_name(int var) const {
  return parent->get_variable_name(var);
}

int PartialStateTask::get_variable_domain_size(int var) const {
  return parent->get_variable_domain_size(var) + 1;
}

int PartialStateTask::get_variable_axiom_layer(int var) const {
  return parent->get_variable_axiom_layer(var);
}

int PartialStateTask::get_variable_default_axiom_value(int var) const {
  return parent->get_variable_default_axiom_value(var);
}

string PartialStateTask::get_fact_name(const FactPair &fact) const {
  if (fact.value == get_variable_domain_size(fact.var) - 1) return "<unknown>";

  return parent->get_fact_name(fact);
}

bool PartialStateTask::are_facts_mutex(const FactPair &fact1,
                                       const FactPair &fact2) const {
  return parent->are_facts_mutex(fact1, fact2);
}

int PartialStateTask::get_operator_cost(int index, bool is_axiom) const {
  return parent->get_operator_cost(index, is_axiom);
}

string PartialStateTask::get_operator_name(int index, bool is_axiom) const {
  return parent->get_operator_name(index, is_axiom);
}

int PartialStateTask::get_num_operators() const {
  return parent->get_num_operators();
}

int PartialStateTask::get_num_operator_preconditions(int index,
                                                     bool is_axiom) const {
  return parent->get_num_operator_preconditions(index, is_axiom);
}

FactPair PartialStateTask::get_operator_precondition(int op_index,
                                                     int fact_index,
                                                     bool is_axiom) const {
  return parent->get_operator_precondition(op_index, fact_index, is_axiom);
}

int PartialStateTask::get_num_operator_effects(int op_index,
                                               bool is_axiom) const {
  return parent->get_num_operator_effects(op_index, is_axiom);
}

int PartialStateTask::get_num_operator_effect_conditions(int op_index,
                                                         int eff_index,
                                                         bool is_axiom) const {
  return parent->get_num_operator_effect_conditions(op_index, eff_index,
                                                    is_axiom);
}

FactPair PartialStateTask::get_operator_effect_condition(int op_index,
                                                         int eff_index,
                                                         int cond_index,
                                                         bool is_axiom) const {
  return parent->get_operator_effect_condition(op_index, eff_index, cond_index,
                                               is_axiom);
}

FactPair PartialStateTask::get_operator_effect(int op_index, int eff_index,
                                               bool is_axiom) const {
  return parent->get_operator_effect(op_index, eff_index, is_axiom);
}

int PartialStateTask::convert_operator_index(
    int index, const AbstractTask *ancestor_task) const {
  if (ancestor_task == this) {
    return index;
  }
  return parent->convert_operator_index(index, ancestor_task);
}

int PartialStateTask::get_num_axioms() const {
  return parent->get_num_axioms();
}

int PartialStateTask::get_num_goals() const { return parent->get_num_goals(); }

FactPair PartialStateTask::get_goal_fact(int index) const {
  return parent->get_goal_fact(index);
}

vector<int> PartialStateTask::get_initial_state_values() const {
  return parent->get_initial_state_values();
}

void PartialStateTask::convert_state_values(
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
    return PartialStateTask::get_partial_state_task();
}

static Plugin<AbstractTask> _plugin("partial_state", _parse);
}  // namespace tasks
