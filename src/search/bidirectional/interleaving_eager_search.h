
#ifndef INTERLEAVING_EAGER_SEARCH_H
#define INTERLEAVING_EAGER_SEARCH_H

#include "../open_list.h"
#include "bidirectional_search.h"

#include <memory>
#include <vector>

class Evaluator;
class PruningMethod;

namespace options {
class OptionParser;
class Options;
}  // namespace options

namespace interleaving_eager_search {
class InterleavingEagerSearch
    : public bidirectional_search::BidirectionalSearch {
  const bool reopen_closed_nodes;

  std::unordered_map<Direction, std::shared_ptr<StateOpenList>> open_lists;
  std::unordered_map<Direction, std::shared_ptr<Evaluator>> f_evaluators;

  std::unordered_map<Direction, std::vector<Evaluator *>>
      path_dependent_evaluators;
  std::unordered_map<Direction, std::vector<std::shared_ptr<Evaluator>>>
      preferred_operator_evaluators;

  std::unordered_map<Direction, std::shared_ptr<PruningMethod>> pruning_methods;

  void start_f_value_statistics(Direction d, EvaluationContext &eval_context);
  void update_f_value_statistics(Direction d, EvaluationContext &eval_context);
  void reward_progress(Direction d);

 protected:
  Direction current_direction;

  virtual void initialize() override;
  virtual SearchStatus step() override;

 public:
  explicit InterleavingEagerSearch(const options::Options &opts);
  virtual ~InterleavingEagerSearch() = default;

  virtual void print_statistics() const override;

  void dump_search_space() const;
};

extern void add_options_to_parser(options::OptionParser &parser);
}  // namespace interleaving_eager_search

#endif
