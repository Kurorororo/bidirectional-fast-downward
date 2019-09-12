#include "bidirectional_search.h"

#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../option_parser.h"
#include "inverse_task.h"

#include <algorithm>
#include <iostream>

using namespace std;

namespace bidirectional_search {

BidirectionalSearch::BidirectionalSearch(const Options &opts)
    : SearchEngine(opts),
      inverse_task(tasks::InverseTask::get_inverse_task()),
      inverse_task_proxy(*inverse_task),
      inverse_succssor_generator(get_successor_generator(inverse_task_proxy)) {}

bool BidirectionalSearch::check_meeting_and_set_plan(
    BidirectionalSearch::Direction d, const GlobalState &parent,
    const GlobalState &state) {
  SearchNode node = search_space.get_node(state);

  if (!node.is_new() && directions[state] != d) {
    cout << "Solution found!" << endl;
    Plan plan1;
    search_space.trace_path(state, plan1);
    Plan plan2;
    search_space.trace_path(parent, plan2);

    if (d == Direction::FORWARD) {
      reverse(plan2.begin(), plan2.end());
      plan1.insert(plan1.end(), plan2.begin(), plan2.end());
      set_plan(plan1);
    } else {
      reverse(plan1.begin(), plan1.end());
      plan2.insert(plan2.end(), plan1.begin(), plan1.end());
      set_plan(plan2);
    }

    return true;
  }

  return false;
}

}  // namespace bidirectional_search