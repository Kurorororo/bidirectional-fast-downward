#include "symbolic_closed.h"

#include "../options/option_parser.h"
#include "../options/options.h"

#include <cmath>

#include <iostream>
#include <string>

namespace symbolic_closed {

SymbolicClosedList::SymbolicClosedList(const TaskProxy &task_proxy)
    : n_bdd_vars(0),
      cudd_init_nodes(16000000),
      cudd_init_cache_size(16000000),
      cudd_init_available_memory(0),
      task_proxy(task_proxy),
      manager(nullptr) {
  Init(task_proxy);
}

BDD SymbolicClosedList::GenerateBDDVar(int var, int value) const {
  BDD res = manager->bddOne();

  for (int v : bdd_index[var]) {
    if (value % 2) {
      res = res * bdd_variables[v];
    } else {
      res = res * (!bdd_variables[v]);
    }
    value /= 2;
  }

  return res;
}

void SymbolicClosedList::Init(const TaskProxy &task_proxy) {
  VariablesProxy variables = task_proxy.get_variables();

  bdd_index.resize(variables.size());

  for (auto var : variables) {
    int var_len = std::ceil(std::log2(var.get_domain_size()));

    for (int j = 0; j < var_len; ++j) {
      bdd_index[var.get_id()].push_back(n_bdd_vars);
      n_bdd_vars += 2;
    }
  }

  manager = std::unique_ptr<Cudd>(
      new Cudd(n_bdd_vars, 0, cudd_init_nodes / n_bdd_vars,
               cudd_init_cache_size, cudd_init_available_memory));

  closed = manager->bddZero();

  for (int i = 0; i < n_bdd_vars; ++i)
    bdd_variables.push_back(manager->bddVar(i));

  var_value_to_bdd.resize(variables.size());

  for (auto var : variables) {
    for (int j = 0; j < var.get_domain_size(); ++j)
      var_value_to_bdd[var.get_id()].push_back(GenerateBDDVar(var.get_id(), j));
  }
}

BDD SymbolicClosedList::GetStateBDD(const GlobalState &state) const {
  BDD res = manager->bddOne();
  VariablesProxy variables = task_proxy.get_variables();
  int n = variables.size();

  for (int i = n - 1; i >= 0; --i) {
    if (state[i] == variables[i].get_domain_size() - 1) continue;

    res = res * var_value_to_bdd[i][state[i]];
  }

  return res;
}

bool SymbolicClosedList::IsClosed(const GlobalState &state) const {
  BDD state_bdd = GetStateBDD(state);

  return state_bdd * closed == state_bdd;
}

bool SymbolicClosedList::IsSubsumed(const GlobalState &state) const {
  BDD state_bdd = GetStateBDD(state);

  return !(state_bdd * closed).IsZero();
}

bool SymbolicClosedList::CloseIfNot(const GlobalState &state) {
  BDD state_bdd = GetStateBDD(state);

  if (state_bdd * closed != state_bdd) {
    closed += state_bdd;
    return true;
  }

  return false;
}

void SymbolicClosedList::Close(const GlobalState &state) {
  BDD state_bdd = GetStateBDD(state);
  closed += state_bdd;
}

}  // namespace symbolic_closed