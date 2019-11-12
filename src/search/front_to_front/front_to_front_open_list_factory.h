#ifndef FRONT_TO_FRONT_OPEN_LIST_FACTORY_H
#define FRONT_TO_FRONT_OPEN_LIST_FACTORY_H

#include "front_to_front_open_list.h"

#include <memory>

class FrontToFrontOpenListFactory {
 public:
  FrontToFrontOpenListFactory() = default;
  virtual ~FrontToFrontOpenListFactory() = default;

  FrontToFrontOpenListFactory(const FrontToFrontOpenListFactory &) = delete;

  virtual std::unique_ptr<FrontToFrontStateOpenList>
  create_state_open_list() = 0;
  virtual std::unique_ptr<FrontToFrontEdgeOpenList> create_edge_open_list() = 0;
  virtual std::unique_ptr<FrontToFrontFrontierOpenList>
  create_frontier_open_list() = 0;
  virtual std::unique_ptr<FrontToFrontFrontierEdgeOpenList>
  create_frontier_edge_open_list() = 0;

  /*
    The following template receives manual specializations (in the
    cc file) for the open list types we want to support. It is
    intended for templatized callers, e.g. the constructor of
    AlternationOpenList.
  */
  template <typename T>
  std::unique_ptr<FrontToFrontOpenList<T>> create_open_list();
};

#endif
