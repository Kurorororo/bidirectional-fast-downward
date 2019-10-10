#include "front_to_front_lifo_open_list.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "front_to_front_heuristic.h"
#include "front_to_front_open_list.h"

#include "../utils/memory.h"

#include <cassert>
#include <deque>
#include <map>
#include <utility>

using namespace std;

namespace front_to_front_lifo_open_list {
template <class Entry>
class FrontToFrontLIFOOpenList : public FrontToFrontOpenList<Entry> {
  typedef deque<Entry> Bucket;

  map<int, Bucket> buckets;
  int size;

  shared_ptr<FrontToFrontHeuristic> evaluator;

 protected:
  virtual void do_insertion(EvaluationContext &eval_context, const Entry &entry,
                            bool to_top) override;

 public:
  explicit FrontToFrontLIFOOpenList(const Options &opts);
  FrontToFrontLIFOOpenList(const shared_ptr<FrontToFrontHeuristic> &eval,
                           bool preferred_only);
  virtual ~FrontToFrontLIFOOpenList() override = default;

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
FrontToFrontLIFOOpenList<Entry>::FrontToFrontLIFOOpenList(const Options &opts)
    : FrontToFrontOpenList<Entry>(opts.get<bool>("pref_only")),
      size(0),
      evaluator(opts.get<shared_ptr<FrontToFrontHeuristic>>("eval")) {}

template <class Entry>
FrontToFrontLIFOOpenList<Entry>::FrontToFrontLIFOOpenList(
    const shared_ptr<FrontToFrontHeuristic> &evaluator, bool preferred_only)
    : FrontToFrontOpenList<Entry>(preferred_only),
      size(0),
      evaluator(evaluator) {}

template <class Entry>
void FrontToFrontLIFOOpenList<Entry>::do_insertion(
    EvaluationContext &eval_context, const Entry &entry, bool to_top) {
  int key = eval_context.get_evaluator_value(evaluator.get());
  buckets[key].push_back(entry);
  ++size;
}

template <class Entry>
Entry FrontToFrontLIFOOpenList<Entry>::remove_min() {
  assert(size > 0);
  auto it = buckets.begin();
  assert(it != buckets.end());
  Bucket &bucket = it->second;
  assert(!bucket.empty());
  Entry result = bucket.back();
  bucket.pop_back();
  if (bucket.empty()) buckets.erase(it);
  --size;
  return result;
}

template <class Entry>
bool FrontToFrontLIFOOpenList<Entry>::empty() const {
  return size == 0;
}

template <class Entry>
void FrontToFrontLIFOOpenList<Entry>::clear() {
  buckets.clear();
  size = 0;
}

template <class Entry>
void FrontToFrontLIFOOpenList<Entry>::get_path_dependent_evaluators(
    set<Evaluator *> &evals) {
  evaluator->get_path_dependent_evaluators(evals);
}

template <class Entry>
bool FrontToFrontLIFOOpenList<Entry>::is_dead_end(
    EvaluationContext &eval_context) const {
  return eval_context.is_evaluator_value_infinite(evaluator.get());
}

template <class Entry>
bool FrontToFrontLIFOOpenList<Entry>::is_reliable_dead_end(
    EvaluationContext &eval_context) const {
  return is_dead_end(eval_context) && evaluator->dead_ends_are_reliable();
}

template <class Entry>
pair<int, Entry> FrontToFrontLIFOOpenList<Entry>::get_min_value_and_entry() {
  assert(size > 0);
  auto it = buckets.begin();
  assert(it != buckets.end());
  Bucket &bucket = it->second;
  assert(!bucket.empty());
  Entry result = bucket.back();

  return make_pair(it->first, result);
}

template <class Entry>
void FrontToFrontLIFOOpenList<Entry>::set_goal(
    const GlobalState &global_state) {
  evaluator->set_goal(global_state);
}

FrontToFrontLIFOOpenListFactory::FrontToFrontLIFOOpenListFactory(
    const Options &options)
    : options(options) {}

unique_ptr<FrontToFrontStateOpenList>
FrontToFrontLIFOOpenListFactory::create_state_open_list() {
  return utils::make_unique_ptr<FrontToFrontLIFOOpenList<StateOpenListEntry>>(
      options);
}

unique_ptr<FrontToFrontEdgeOpenList>
FrontToFrontLIFOOpenListFactory::create_edge_open_list() {
  return utils::make_unique_ptr<FrontToFrontLIFOOpenList<EdgeOpenListEntry>>(
      options);
}

unique_ptr<FrontToFrontFrontierOpenList>
FrontToFrontLIFOOpenListFactory::create_frontier_open_list() {
  return utils::make_unique_ptr<
      FrontToFrontLIFOOpenList<FrontierOpenListEntry>>(options);
}

static shared_ptr<FrontToFrontOpenListFactory> _parse(OptionParser &parser) {
  parser.document_synopsis(
      "Front to front open list",
      "Open list that uses a single evaluator and FIFO tiebreaking.");
  parser.document_note(
      "Implementation Notes",
      "Elements with the same evaluator value are stored in double-ended "
      "queues, called \"buckets\". The open list stores a map from evaluator "
      "values to buckets. Pushing and popping from a bucket runs in constant "
      "time. Therefore, inserting and removing an entry from the open list "
      "takes time O(log(n)), where n is the number of buckets.");
  parser.add_option<shared_ptr<FrontToFrontHeuristic>>(
      "eval", "front_to_front_heuristic");
  parser.add_option<bool>("pref_only",
                          "insert only nodes generated by preferred operators",
                          "false");

  Options opts = parser.parse();
  if (parser.dry_run())
    return nullptr;
  else
    return make_shared<FrontToFrontLIFOOpenListFactory>(opts);
}

static Plugin<FrontToFrontOpenListFactory> _plugin("front_to_front_lifo",
                                                   _parse);
}  // namespace front_to_front_lifo_open_list
