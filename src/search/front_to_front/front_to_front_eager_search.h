#ifndef FRONT_TO_FRONT_EAGER_SEARCH_H
#define FRONT_TO_FRONT_EAGER_SEARCH_H

#include "../bidirectional/bidirectional_search.h"
#include "front_to_front_open_list.h"

#include <memory>
#include <vector>

class Evaluator;
class PruningMethod;

namespace options {
class OptionParser;
class Options;
}  // namespace options

namespace front_to_front_eager_search {
class FrontToFrontEagerSearch
    : public bidirectional_search::BidirectionalSearch {
  const bool reopen_closed_nodes;

  std::unordered_map<Direction, std::shared_ptr<FrontToFrontStateOpenList>>
      open_lists;
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
  explicit FrontToFrontEagerSearch(const options::Options &opts);
  virtual ~FrontToFrontEagerSearch() = default;

  virtual void print_statistics() const override;

  void dump_search_space() const;
};

extern void add_options_to_parser(options::OptionParser &parser);
}  // namespace front_to_front_eager_search

#endif
