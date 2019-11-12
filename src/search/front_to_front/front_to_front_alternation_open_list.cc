#include "front_to_front_alternation_open_list.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "front_to_front_open_list.h"

#include "../utils/memory.h"
#include "../utils/system.h"

#include <cassert>
#include <memory>
#include <vector>

using namespace std;
using utils::ExitCode;

namespace front_to_front_alternation_open_list {
template <class Entry>
class FrontToFrontAlternationOpenList : public FrontToFrontOpenList<Entry> {
  vector<unique_ptr<FrontToFrontOpenList<Entry>>> open_lists;
  vector<int> priorities;

  const int boost_amount;

 protected:
  virtual void do_insertion(EvaluationContext &eval_context, const Entry &entry,
                            bool to_top) override;

 public:
  explicit FrontToFrontAlternationOpenList(const Options &opts);
  virtual ~FrontToFrontAlternationOpenList() override = default;

  virtual Entry remove_min() override;
  virtual bool empty() const override;
  virtual void clear() override;
  virtual void boost_preferred() override;
  virtual void get_path_dependent_evaluators(set<Evaluator *> &evals) override;
  virtual bool is_dead_end(EvaluationContext &eval_context) const override;
  virtual bool is_reliable_dead_end(
      EvaluationContext &eval_context) const override;
  virtual pair<int, Entry> get_min_value_and_entry() override;
  virtual void set_goal(const GlobalState &global_state) override;
};

template <class Entry>
FrontToFrontAlternationOpenList<Entry>::FrontToFrontAlternationOpenList(
    const Options &opts)
    : boost_amount(opts.get<int>("boost")) {
  vector<shared_ptr<FrontToFrontOpenListFactory>> open_list_factories(
      opts.get_list<shared_ptr<FrontToFrontOpenListFactory>>("sublists"));
  open_lists.reserve(open_list_factories.size());
  for (const auto &factory : open_list_factories)
    open_lists.push_back(factory->create_open_list<Entry>());

  priorities.resize(open_lists.size(), 0);
}

template <class Entry>
void FrontToFrontAlternationOpenList<Entry>::do_insertion(
    EvaluationContext &eval_context, const Entry &entry, bool to_top) {
  for (const auto &sublist : open_lists)
    sublist->insert(eval_context, entry, to_top);
}

template <class Entry>
Entry FrontToFrontAlternationOpenList<Entry>::remove_min() {
  int best = -1;
  for (size_t i = 0; i < open_lists.size(); ++i) {
    if (!open_lists[i]->empty() &&
        (best == -1 || priorities[i] < priorities[best])) {
      best = i;
    }
  }
  assert(best != -1);
  const auto &best_list = open_lists[best];
  assert(!best_list->empty());
  ++priorities[best];
  return best_list->remove_min();
}

template <class Entry>
bool FrontToFrontAlternationOpenList<Entry>::empty() const {
  for (const auto &sublist : open_lists)
    if (!sublist->empty()) return false;
  return true;
}

template <class Entry>
void FrontToFrontAlternationOpenList<Entry>::clear() {
  for (const auto &sublist : open_lists) sublist->clear();
}

template <class Entry>
void FrontToFrontAlternationOpenList<Entry>::boost_preferred() {
  for (size_t i = 0; i < open_lists.size(); ++i)
    if (open_lists[i]->only_contains_preferred_entries())
      priorities[i] -= boost_amount;
}

template <class Entry>
void FrontToFrontAlternationOpenList<Entry>::get_path_dependent_evaluators(
    set<Evaluator *> &evals) {
  for (const auto &sublist : open_lists)
    sublist->get_path_dependent_evaluators(evals);
}

template <class Entry>
bool FrontToFrontAlternationOpenList<Entry>::is_dead_end(
    EvaluationContext &eval_context) const {
  // If one sublist is sure we have a dead end, return true.
  if (is_reliable_dead_end(eval_context)) return true;
  // Otherwise, return true if all sublists agree this is a dead-end.
  for (const auto &sublist : open_lists)
    if (!sublist->is_dead_end(eval_context)) return false;
  return true;
}

template <class Entry>
bool FrontToFrontAlternationOpenList<Entry>::is_reliable_dead_end(
    EvaluationContext &eval_context) const {
  for (const auto &sublist : open_lists)
    if (sublist->is_reliable_dead_end(eval_context)) return true;
  return false;
}

template <class Entry>
pair<int, Entry>
FrontToFrontAlternationOpenList<Entry>::get_min_value_and_entry() {
  int best = -1;
  for (size_t i = 0; i < open_lists.size(); ++i) {
    if (!open_lists[i]->empty() &&
        (best == -1 || priorities[i] < priorities[best])) {
      best = i;
    }
  }
  assert(best != -1);
  const auto &best_list = open_lists[best];
  assert(!best_list->empty());
  return best_list->get_min_value_and_entry();
}

template <class Entry>
void FrontToFrontAlternationOpenList<Entry>::set_goal(
    const GlobalState &global_state) {
  for (const auto &sublist : open_lists) sublist->set_goal(global_state);
}

FrontToFrontAlternationOpenListFactory::FrontToFrontAlternationOpenListFactory(
    const Options &options)
    : options(options) {}

unique_ptr<FrontToFrontStateOpenList>
FrontToFrontAlternationOpenListFactory::create_state_open_list() {
  return utils::make_unique_ptr<
      FrontToFrontAlternationOpenList<StateOpenListEntry>>(options);
}

unique_ptr<FrontToFrontEdgeOpenList>
FrontToFrontAlternationOpenListFactory::create_edge_open_list() {
  return utils::make_unique_ptr<
      FrontToFrontAlternationOpenList<EdgeOpenListEntry>>(options);
}

unique_ptr<FrontToFrontFrontierOpenList>
FrontToFrontAlternationOpenListFactory::create_frontier_open_list() {
  return utils::make_unique_ptr<
      FrontToFrontAlternationOpenList<FrontierOpenListEntry>>(options);
}

unique_ptr<FrontToFrontFrontierEdgeOpenList>
FrontToFrontAlternationOpenListFactory::create_frontier_edge_open_list() {
  return utils::make_unique_ptr<
      FrontToFrontAlternationOpenList<FrontierEdgeOpenListEntry>>(options);
}

static shared_ptr<FrontToFrontOpenListFactory> _parse(OptionParser &parser) {
  parser.document_synopsis("FrontToFrontAlternation open list",
                           "alternates between several open lists.");
  parser.add_list_option<shared_ptr<FrontToFrontOpenListFactory>>(
      "sublists", "open lists between which this one alternates");
  parser.add_option<int>(
      "boost",
      "boost value for contained open lists that are restricted "
      "to preferred successors",
      "0");

  Options opts = parser.parse();
  opts.verify_list_non_empty<shared_ptr<FrontToFrontOpenListFactory>>(
      "sublists");
  if (parser.dry_run())
    return nullptr;
  else
    return make_shared<FrontToFrontAlternationOpenListFactory>(opts);
}

static Plugin<FrontToFrontOpenListFactory> _plugin("front_to_front_alt",
                                                   _parse);
}  // namespace front_to_front_alternation_open_list
