#include "front_to_front_tiebreaking_open_list.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "front_to_front_heuristic.h"
#include "front_to_front_open_list.h"

#include "../utils/memory.h"

#include <cassert>
#include <deque>
#include <map>
#include <utility>
#include <vector>

using namespace std;

namespace front_to_front_tiebreaking_open_list {
template <class Entry>
class FrontToFrontTieBreakingOpenList : public FrontToFrontOpenList<Entry> {
  using Bucket = deque<Entry>;

  map<const vector<int>, Bucket> buckets;
  int size;

  vector<shared_ptr<FrontToFrontHeuristic>> evaluators;
  /*
    If allow_unsafe_pruning is true, we ignore (don't insert) states
    which the first evaluator considers a dead end, even if it is
    not a safe heuristic.
  */
  bool allow_unsafe_pruning;

  int dimension() const;

 protected:
  virtual void do_insertion(EvaluationContext &eval_context, const Entry &entry,
                            bool to_top) override;

 public:
  explicit FrontToFrontTieBreakingOpenList(const Options &opts);
  virtual ~FrontToFrontTieBreakingOpenList() override = default;

  virtual Entry remove_min() override;
  virtual bool empty() const override;
  virtual void clear() override;
  virtual void get_path_dependent_evaluators(set<Evaluator *> &evals) override;
  virtual bool is_dead_end(EvaluationContext &eval_context) const override;
  virtual bool is_reliable_dead_end(
      EvaluationContext &eval_context) const override;
  virtual pair<int, Entry> get_min_value_and_entry() override;
  virtual void set_goal(const GlobalState &global_state) override;
};

template <class Entry>
FrontToFrontTieBreakingOpenList<Entry>::FrontToFrontTieBreakingOpenList(
    const Options &opts)
    : FrontToFrontOpenList<Entry>(opts.get<bool>("pref_only")),
      size(0),
      evaluators(opts.get_list<shared_ptr<FrontToFrontHeuristic>>("evals")),
      allow_unsafe_pruning(opts.get<bool>("unsafe_pruning")) {}

template <class Entry>
void FrontToFrontTieBreakingOpenList<Entry>::do_insertion(
    EvaluationContext &eval_context, const Entry &entry, bool to_top) {
  vector<int> key;
  key.reserve(evaluators.size());
  for (const shared_ptr<Evaluator> &evaluator : evaluators)
    key.push_back(
        eval_context.get_evaluator_value_or_infinity(evaluator.get()));

  if (to_top)
    buckets[key].push_front(entry);
  else
    buckets[key].push_back(entry);
  ++size;
}

template <class Entry>
Entry FrontToFrontTieBreakingOpenList<Entry>::remove_min() {
  assert(size > 0);
  typename map<const vector<int>, Bucket>::iterator it;
  it = buckets.begin();
  assert(it != buckets.end());
  assert(!it->second.empty());
  --size;
  Entry result = it->second.front();
  it->second.pop_front();
  if (it->second.empty()) buckets.erase(it);
  return result;
}

template <class Entry>
bool FrontToFrontTieBreakingOpenList<Entry>::empty() const {
  return size == 0;
}

template <class Entry>
void FrontToFrontTieBreakingOpenList<Entry>::clear() {
  buckets.clear();
  size = 0;
}

template <class Entry>
int FrontToFrontTieBreakingOpenList<Entry>::dimension() const {
  return evaluators.size();
}

template <class Entry>
void FrontToFrontTieBreakingOpenList<Entry>::get_path_dependent_evaluators(
    set<Evaluator *> &evals) {
  for (const shared_ptr<Evaluator> &evaluator : evaluators)
    evaluator->get_path_dependent_evaluators(evals);
}

template <class Entry>
bool FrontToFrontTieBreakingOpenList<Entry>::is_dead_end(
    EvaluationContext &eval_context) const {
  // TODO: Properly document this behaviour.
  // If one safe heuristic detects a dead end, return true.
  if (is_reliable_dead_end(eval_context)) return true;
  // If the first heuristic detects a dead-end and we allow "unsafe
  // pruning", return true.
  if (allow_unsafe_pruning &&
      eval_context.is_evaluator_value_infinite(evaluators[0].get()))
    return true;
  // Otherwise, return true if all heuristics agree this is a dead-end.
  for (const shared_ptr<Evaluator> &evaluator : evaluators)
    if (!eval_context.is_evaluator_value_infinite(evaluator.get()))
      return false;
  return true;
}

template <class Entry>
bool FrontToFrontTieBreakingOpenList<Entry>::is_reliable_dead_end(
    EvaluationContext &eval_context) const {
  for (const shared_ptr<Evaluator> &evaluator : evaluators)
    if (eval_context.is_evaluator_value_infinite(evaluator.get()) &&
        evaluator->dead_ends_are_reliable())
      return true;
  return false;
}

template <class Entry>
pair<int, Entry>
FrontToFrontTieBreakingOpenList<Entry>::get_min_value_and_entry() {
  assert(size > 0);
  auto it = buckets.begin();
  assert(it != buckets.end());
  Bucket &bucket = it->second;
  assert(!bucket.empty());
  Entry result = bucket.front();

  return make_pair(it->first[0], result);
}

template <class Entry>
void FrontToFrontTieBreakingOpenList<Entry>::set_goal(
    const GlobalState &global_state) {
  for (const shared_ptr<FrontToFrontHeuristic> &evaluator : evaluators)
    evaluator->set_goal(global_state);
}

FrontToFrontTieBreakingOpenListFactory::FrontToFrontTieBreakingOpenListFactory(
    const Options &options)
    : options(options) {}

unique_ptr<FrontToFrontStateOpenList>
FrontToFrontTieBreakingOpenListFactory::create_state_open_list() {
  return utils::make_unique_ptr<
      FrontToFrontTieBreakingOpenList<StateOpenListEntry>>(options);
}

unique_ptr<FrontToFrontEdgeOpenList>
FrontToFrontTieBreakingOpenListFactory::create_edge_open_list() {
  return utils::make_unique_ptr<
      FrontToFrontTieBreakingOpenList<EdgeOpenListEntry>>(options);
}

unique_ptr<FrontToFrontFrontierOpenList>
FrontToFrontTieBreakingOpenListFactory::create_frontier_open_list() {
  return utils::make_unique_ptr<
      FrontToFrontTieBreakingOpenList<FrontierOpenListEntry>>(options);
}

static shared_ptr<FrontToFrontOpenListFactory> _parse(OptionParser &parser) {
  parser.document_synopsis("Tie-breaking open list", "");
  parser.add_list_option<shared_ptr<FrontToFrontHeuristic>>("evals",
                                                            "evaluators");
  parser.add_option<bool>("pref_only",
                          "insert only nodes generated by preferred operators",
                          "false");
  parser.add_option<bool>(
      "unsafe_pruning",
      "allow unsafe pruning when the main evaluator regards a state a dead end",
      "true");
  Options opts = parser.parse();
  opts.verify_list_non_empty<shared_ptr<FrontToFrontHeuristic>>("evals");
  if (parser.dry_run())
    return nullptr;
  else
    return make_shared<FrontToFrontTieBreakingOpenListFactory>(opts);
}

static Plugin<FrontToFrontOpenListFactory> _plugin("front_to_front_tiebreaking",
                                                   _parse);
}  // namespace front_to_front_tiebreaking_open_list
