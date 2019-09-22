#ifndef FRONT_TO_FRONT_FF_HEURISTIC_H
#define FRONT_TO_FRONT_FF_HEURISTIC_H

#include "front_to_front_additive_heuristic.h"

#include <vector>

namespace front_to_front_ff_heuristic {
using front_to_front_relaxation_heuristic::OpID;
using front_to_front_relaxation_heuristic::PropID;

using front_to_front_relaxation_heuristic::NO_OP;

using front_to_front_relaxation_heuristic::Proposition;
using front_to_front_relaxation_heuristic::UnaryOperator;

/*
  TODO: In a better world, this should not derive from
        AdditiveHeuristic. Rather, the common parts should be
        implemented in a common base class. That refactoring could be
        made at the same time at which we also unify this with the
        other relaxation heuristics and the additional FF heuristic
        implementation in the landmark code.
*/
class FrontToFrontFFHeuristic
    : public front_to_front_additive_heuristic::FrontToFrontAdditiveHeuristic {
  // Relaxed plans are represented as a set of operators implemented
  // as a bit vector.
  using RelaxedPlan = std::vector<bool>;
  RelaxedPlan relaxed_plan;
  void mark_preferred_operators_and_relaxed_plan(const State &state,
                                                 PropID goal_id);

 protected:
  virtual int compute_heuristic(const GlobalState &global_state) override;

 public:
  explicit FrontToFrontFFHeuristic(const options::Options &opts);
};
}  // namespace front_to_front_ff_heuristic

#endif
