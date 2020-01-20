#ifndef FRONT_TO_FRONT_MAX_HEURISTIC_H
#define FRONT_TO_FRONT_MAX_HEURISTIC_H

#include "front_to_front_relaxation_heuristic.h"

#include "../algorithms/priority_queues.h"

#include <cassert>

namespace front_to_front_max_heuristic {
using front_to_front_relaxation_heuristic::OpID;
using front_to_front_relaxation_heuristic::PropID;

using front_to_front_relaxation_heuristic::Proposition;
using front_to_front_relaxation_heuristic::UnaryOperator;

class FrontToFrontHSPMaxHeuristic : public front_to_front_relaxation_heuristic::
                                        FrontToFrontRelaxationHeuristic {
  priority_queues::AdaptiveQueue<PropID> queue;

  void setup_exploration_queue();
  void setup_exploration_queue_state(const State &state);
  void relaxed_exploration();

  void enqueue_if_necessary(PropID prop_id, int cost) {
    assert(cost >= 0);
    Proposition *prop = get_proposition(prop_id);
    if (prop->cost == -1 || prop->cost > cost) {
      prop->cost = cost;
      queue.push(cost, prop_id);
    }
    assert(prop->cost != -1 && prop->cost <= cost);
  }

  void precompute_exploration(const State &state);

 public:
  enum FallBackTo { NONE = 0, INITIAL = 1, GOAL = 2 };

 protected:
  bool cache_initial;
  FallBackTo fall_back_to;

  virtual int compute_heuristic(const GlobalState &global_state) override;

 public:
  explicit FrontToFrontHSPMaxHeuristic(const options::Options &opts);
};
}  // namespace front_to_front_max_heuristic

#endif
