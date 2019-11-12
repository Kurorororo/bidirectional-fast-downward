#ifndef FRONT_TO_FRONT_LIFO_OPEN_LIST_H
#define FRONT_TO_FRONT_LIFO_OPEN_LIST_H

#include "../option_parser_util.h"
#include "front_to_front_open_list_factory.h"

/*
  Open list indexed by a single int, using FIFO tie-breaking.

  Implemented as a map from int to deques.
*/

namespace front_to_front_lifo_open_list {
class FrontToFrontLIFOOpenListFactory : public FrontToFrontOpenListFactory {
  Options options;

 public:
  explicit FrontToFrontLIFOOpenListFactory(const Options &options);
  virtual ~FrontToFrontLIFOOpenListFactory() override = default;

  virtual std::unique_ptr<FrontToFrontStateOpenList> create_state_open_list()
      override;
  virtual std::unique_ptr<FrontToFrontEdgeOpenList> create_edge_open_list()
      override;
  virtual std::unique_ptr<FrontToFrontFrontierOpenList>
  create_frontier_open_list() override;
  virtual std::unique_ptr<FrontToFrontFrontierEdgeOpenList>
  create_frontier_edge_open_list() override;
};
}  // namespace front_to_front_lifo_open_list

#endif
