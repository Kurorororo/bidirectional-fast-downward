#ifndef TASKS_ROOT_TASK_H
#define TASKS_ROOT_TASK_H

#include "../abstract_task.h"

namespace tasks {
extern std::shared_ptr<AbstractTask> g_root_task;
extern void read_root_task(std::istream &in);

struct ExplicitEffect {
  FactPair fact;
  std::vector<FactPair> conditions;

  ExplicitEffect(int var, int value, std::vector<FactPair> &&conditions);
};

struct ExplicitOperator {
  std::vector<FactPair> preconditions;
  std::vector<ExplicitEffect> effects;
  int cost;
  std::string name;
  bool is_an_axiom;

  void read_pre_post(std::istream &in);
  ExplicitOperator(std::istream &in, bool is_an_axiom, bool use_metric);
  ExplicitOperator(std::vector<FactPair> &&preconditions,
                   std::vector<ExplicitEffect> &&effects, int cost,
                   const std::string &name, bool is_an_axiom);
};

}  // namespace tasks
#endif
