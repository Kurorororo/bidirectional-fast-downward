#include "regression_state_registry.h"

using namespace std;

void RegressionStateRegistry::init_mutex() {
  TaskProxy task_proxy = get_task_proxy();
  VariablesProxy variables = task_proxy.get_variables();

  fact_to_mutexes.resize(variables.size(), vector<vector<pair<int, int>>>());

  for (int var1 = 0, n = variables.size(); var1 < n; ++var1)
    fact_to_mutexes[var1].resize(variables[var1].get_domain_size(),
                                 vector<pair<int, int>>());

  for (int var1 = 0, n = variables.size(); var1 < n; ++var1) {
    for (int value1 = 0, m = variables[var1].get_domain_size() - 1; value1 < m;
         ++value1) {
      FactProxy fact1 = variables[var1].get_fact(value1);

      for (int var2 = var1 + 1, l = variables.size(); var2 < l; ++var2) {
        for (int value2 = 0, o = variables[var2].get_domain_size() - 1;
             value2 < o; ++value2) {
          FactProxy fact2 = variables[var2].get_fact(value2);

          if (fact1.is_mutex(fact2)) {
            fact_to_mutexes[var1][value1].push_back(make_pair(var2, value2));
            fact_to_mutexes[var2][value2].push_back(make_pair(var1, value1));
          }
        }
      }
    }
  }
}

RegressionStateRegistry::RegressionStateRegistry(const TaskProxy &task_proxy)
    : StateRegistry(task_proxy) {
  init_mutex();
}

RegressionStateRegistry::~RegressionStateRegistry() {}

StateID RegressionStateRegistry::get_predecessor_state(
    const GlobalState &successor, const OperatorProxy &op) {
  TaskProxy task_proxy = get_task_proxy();

  state_data_pool.push_back(get_packed_buffer(successor));
  PackedStateBin *buffer = state_data_pool[state_data_pool.size() - 1];
  for (EffectProxy effect : op.get_effects()) {
    if (does_fire(effect, successor)) {
      FactPair effect_pair = effect.get_fact().get_pair();
      state_packer.set(buffer, effect_pair.var, effect_pair.value);
    }
  }

  VariablesProxy variables = task_proxy.get_variables();
  bool has_mutex = false;
  bool has_undefined = false;

  for (auto var : variables) {
    int value = state_packer.get(buffer, var.get_id());

    if (value == var.get_domain_size() - 1)
      has_mutex = true;
    else if (!fact_to_mutexes[var.get_id()][value].empty())
      has_undefined = true;

    if (has_mutex && has_undefined) break;
  }

  if (!has_mutex && !has_undefined) return insert_id_or_pop_state();

  vector<unordered_set<int>> ranges(variables.size(), unordered_set<int>());

  for (auto var : variables)
    for (int value = 0; value < var.get_domain_size() - 1; ++value)
      ranges[var.get_id()].insert(value);

  bool invalid = false;

  for (auto var : variables) {
    if (invalid) break;

    int value = state_packer.get(buffer, var.get_id());

    if (value == var.get_domain_size() - 1) continue;

    for (auto fact : fact_to_mutexes[var.get_id()][value]) {
      if (state_packer.get(buffer, fact.first) == fact.second) {
        invalid = true;
        break;
      }

      auto result = ranges[fact.first].find(fact.second);
      if (result != ranges[fact.first].end()) ranges[fact.first].erase(result);

      if (ranges[fact.first].empty()) {
        invalid = true;
        break;
      }
    }
  }

  if (invalid) {
    state_data_pool.pop_back();
    return StateID::no_state;
  }

  for (auto var : variables) {
    int value = state_packer.get(buffer, var.get_id());

    if (value != var.get_domain_size() - 1) continue;

    if (ranges[var.get_id()].size() == 1)
      state_packer.set(buffer, var.get_id(), *ranges[var.get_id()].begin());
  }

  return insert_id_or_pop_state();
}