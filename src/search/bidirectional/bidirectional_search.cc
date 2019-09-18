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
      inverse_successor_generator(get_successor_generator(inverse_task_proxy)),
      directions(Direction::NONE) {}

BidirectionalSearch::~BidirectionalSearch() {
  // for (auto a : get_plan()) {
  //  OperatorProxy op = task_proxy.get_operators()[a];
  //  OperatorProxy inverse_op = inverse_task_proxy.get_operators()[a];

  //  cout << endl << op.get_name() << endl;

  //  cout << "forward preconditions" << endl;

  //  for (auto f : op.get_preconditions()) {
  //    cout << f.get_name() << endl;
  //  }

  //  cout << "backward preconditions" << endl;

  //  for (auto f : inverse_op.get_preconditions()) {
  //    cout << f.get_name() << endl;
  //  }

  //  cout << "forward effects" << endl;

  //  for (auto f : op.get_effects()) {
  //    cout << f.get_fact().get_name() << endl;
  //  }

  //  cout << "backward effects" << endl;

  //  for (auto f : inverse_op.get_effects()) {
  //    cout << f.get_fact().get_name() << endl;
  //  }
  //}
}

bool BidirectionalSearch::check_meeting_and_set_plan(
    BidirectionalSearch::Direction d, const GlobalState &parent,
    OperatorID op_id, const GlobalState &state) {
  SearchNode node = search_space.get_node(state);

  if (!node.is_new() && directions[state] != Direction::NONE &&
      directions[state] != d) {
    cout << "Solution found!" << endl;

    Plan plan1;
    search_space.trace_path(state, plan1);
    Plan plan2;
    search_space.trace_path(parent, plan2);

    if (d == Direction::FORWARD) {
      cout << "#forward actions: " << plan2.size() + 1 << endl;
      cout << "#backward actions: " << plan1.size() << endl;
      reverse(plan1.begin(), plan1.end());
      plan2.push_back(op_id);
      plan2.insert(plan2.end(), plan1.begin(), plan1.end());
      set_plan(plan2);
    } else {
      cout << "#forward actions: " << plan1.size() << endl;
      cout << "#backward actions: " << plan2.size() + 1 << endl;
      reverse(plan2.begin(), plan2.end());
      plan1.push_back(op_id);
      plan1.insert(plan1.end(), plan2.begin(), plan2.end());
      set_plan(plan1);
    }

    return true;
  }

  return false;
}

}  // namespace bidirectional_search