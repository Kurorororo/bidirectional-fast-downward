#ifndef BIDIRECTIONAL_SEARCH_H
#define BIDIRECTIONAL_SEARCH_H

#include "../operator_cost.h"
#include "../operator_id.h"
#include "../plan_manager.h"
#include "../search_engine.h"
#include "../search_progress.h"
#include "../search_space.h"
#include "../search_statistics.h"
#include "../state_registry.h"
#include "../task_proxy.h"

namespace bidirectional_search {
class BidirectionalSearch : public SearchEngine {
 protected:
  enum Direction { FORWARD = 0, BACKWARD = 1 };
  const std::shared_ptr<AbstractTask> inverse_task;
  TaskProxy inverse_task_proxy;
  const successor_generator::SuccessorGenerator &inverse_succssor_generator;
  PerStateInformation<Direction> directions;

  bool check_meeting_and_set_plan(Direction d, const GlobalState &parent,
                                  const GlobalState &state);

 public:
  BidirectionalSearch(const options::Options &opts);
  virtual ~BidirectionalSearch();
};
}  // namespace bidirectional_search

#endif