#ifndef FRONT_TO_FRONT_BEST_FIRST_OPEN_LIST_H
#define FRONT_TO_FRONT_BEST_FIRST_OPEN_LIST_H

#include "../option_parser_util.h"
#include "front_to_front_open_list_factory.h"

/*
  Open list indexed by a single int, using FIFO tie-breaking.

  Implemented as a map from int to deques.
*/

namespace front_to_front_best_first_open_list {
class FrontToFrontBestFirstOpenListFactory
    : public FrontToFrontOpenListFactory {
  Options options;

 public:
  explicit FrontToFrontBestFirstOpenListFactory(const Options &options);
  virtual ~FrontToFrontBestFirstOpenListFactory() override = default;

  virtual std::unique_ptr<FrontToFrontStateOpenList> create_state_open_list()
      override;
  virtual std::unique_ptr<FrontToFrontEdgeOpenList> create_edge_open_list()
      override;
};
}  // namespace front_to_front_best_first_open_list

#endif
