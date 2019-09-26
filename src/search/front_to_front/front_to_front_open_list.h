#ifndef FRONT_TO_FRONT_OPEN_LIST_H
#define FRONT_TO_FRONT_OPEN_LIST_H

#include <iostream>
#include <set>
#include <utility>

#include "../evaluation_context.h"
#include "../operator_id.h"

class StateID;

template <class Entry>
class FrontToFrontOpenList {
  bool only_preferred;

 protected:
  virtual void do_insertion(EvaluationContext &eval_context,
                            const Entry &entry) = 0;

 public:
  explicit FrontToFrontOpenList(bool preferred_only = false);
  virtual ~FrontToFrontOpenList() = default;

  void insert(EvaluationContext &eval_context, const Entry &entry);
  virtual Entry remove_min() = 0;
  virtual bool empty() const = 0;
  virtual void clear() = 0;
  virtual void boost_preferred();
  virtual void get_path_dependent_evaluators(std::set<Evaluator *> &evals) = 0;
  bool only_contains_preferred_entries() const;
  virtual bool is_dead_end(EvaluationContext &eval_context) const = 0;
  virtual bool is_reliable_dead_end(EvaluationContext &eval_context) const = 0;

  virtual std::pair<int, Entry> get_min_value_and_entry() = 0;
  virtual void set_goal(const GlobalState &global_state) = 0;
};

using StateOpenListEntry = StateID;
using EdgeOpenListEntry = std::pair<StateID, OperatorID>;
using FrontierOpenListEntry = std::pair<StateID, StateID>;

using FrontToFrontStateOpenList = FrontToFrontOpenList<StateOpenListEntry>;
using FrontToFrontEdgeOpenList = FrontToFrontOpenList<EdgeOpenListEntry>;
using FrontToFrontFrontierOpenList =
    FrontToFrontOpenList<FrontierOpenListEntry>;

template <class Entry>
FrontToFrontOpenList<Entry>::FrontToFrontOpenList(bool only_preferred)
    : only_preferred(only_preferred) {}

template <class Entry>
void FrontToFrontOpenList<Entry>::boost_preferred() {}

template <class Entry>
void FrontToFrontOpenList<Entry>::insert(EvaluationContext &eval_context,
                                         const Entry &entry) {
  if (only_preferred && !eval_context.is_preferred()) return;
  if (!is_dead_end(eval_context)) do_insertion(eval_context, entry);
}

template <class Entry>
bool FrontToFrontOpenList<Entry>::only_contains_preferred_entries() const {
  return only_preferred;
}

#endif