#ifndef FRONT_TO_FRONT_TIEBREAKING_OPEN_LIST_H
#define FRONT_TO_FRONT_TIEBREAKING_OPEN_LIST_H

#include "../option_parser_util.h"
#include "front_to_front_open_list_factory.h"

namespace front_to_front_tiebreaking_open_list {
class FrontToFrontTieBreakingOpenListFactory
    : public FrontToFrontOpenListFactory {
  Options options;

 public:
  explicit FrontToFrontTieBreakingOpenListFactory(const Options &options);
  virtual ~FrontToFrontTieBreakingOpenListFactory() override = default;

  virtual std::unique_ptr<FrontToFrontStateOpenList> create_state_open_list()
      override;
  virtual std::unique_ptr<FrontToFrontEdgeOpenList> create_edge_open_list()
      override;
  virtual std::unique_ptr<FrontToFrontFrontierOpenList>
  create_frontier_open_list() override;
  virtual std::unique_ptr<FrontToFrontFrontierEdgeOpenList>
  create_frontier_edge_open_list() override;
};
}  // namespace front_to_front_tiebreaking_open_list

#endif
