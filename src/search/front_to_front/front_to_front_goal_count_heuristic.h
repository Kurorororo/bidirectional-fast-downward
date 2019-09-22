#ifndef FRONT_TO_FRONT_GOAL_COUNT_HEURISTIC_H
#define FRONT_TO_FRONT_GOAL_COUNT_HEURISTIC_H

#include "../global_state.h"
#include "../state_id.h"
#include "front_to_front_heuristic.h"

namespace front_to_front_goal_count_heuristic {
class FrontToFrontGoalCountHeuristic : public FrontToFrontHeuristic {
 protected:
  virtual int compute_heuristic(const GlobalState &global_state) override;

 public:
  explicit FrontToFrontGoalCountHeuristic(const options::Options &opts);
};
}  // namespace front_to_front_goal_count_heuristic

#endif
