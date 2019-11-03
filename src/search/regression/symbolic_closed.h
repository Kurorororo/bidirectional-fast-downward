#ifndef SYMBOLIC_CLOSED_H_
#define SYMBOLIC_CLOSED_H_

#include "../global_state.h"
#include "../options/option_parser.h"
#include "../options/options.h"
#include "../task_proxy.h"

#include "cuddObj.hh"

#include <memory>
#include <vector>

namespace symbolic_closed {

class SymbolicClosedList {
  void Init(const TaskProxy &task_proxy);

 public:
  SymbolicClosedList(const TaskProxy &task_proxy);
  ~SymbolicClosedList() {}

  BDD GenerateBDDVar(int var, int value) const;

  BDD GetStateBDD(const GlobalState &state) const;

  bool IsClosed(const GlobalState &state) const;

  bool IsSubsumed(const GlobalState &state) const;

  bool CloseIfNot(const GlobalState &state);

  void Close(const GlobalState &state);

 private:
  int n_bdd_vars;
  long cudd_init_nodes;
  long cudd_init_cache_size;
  long cudd_init_available_memory;
  TaskProxy task_proxy;
  std::unique_ptr<Cudd> manager;
  std::vector<std::vector<int>> bdd_index;
  std::vector<BDD> bdd_variables;
  std::vector<std::vector<BDD>> var_value_to_bdd;
  BDD closed;
};

}  // namespace symbolic_closed

#endif  // SYMBOLIC_CLOSED_H_