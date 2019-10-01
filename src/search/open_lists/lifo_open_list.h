#ifndef OPEN_LISTS_LIFO_OPEN_LIST_H
#define OPEN_LISTS_LIFO_OPEN_LIST_H

#include "../open_list_factory.h"
#include "../option_parser_util.h"

/*
  Open list indexed by a single int, using FIFO tie-breaking.

  Implemented as a map from int to deques.
*/

namespace standard_scalar_open_list {
class LIFOOpenListFactory : public OpenListFactory {
  Options options;

 public:
  explicit LIFOOpenListFactory(const Options &options);
  virtual ~LIFOOpenListFactory() override = default;

  virtual std::unique_ptr<StateOpenList> create_state_open_list() override;
  virtual std::unique_ptr<EdgeOpenList> create_edge_open_list() override;
};
}  // namespace standard_scalar_open_list

#endif
