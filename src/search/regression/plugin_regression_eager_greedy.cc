#include "../search_engines/search_common.h"
#include "regression_eager_search.h"

#include "../front_to_front/front_to_front_open_list_factory.h"
#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace plugin_regression_eager_greedy {
static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
  parser.document_synopsis("Regression greedy search (eager)", "");
  parser.document_note(
      "Open list",
      "In most cases, eager greedy best first search uses "
      "an alternation open list with one queue for each evaluator. "
      "If preferred operator evaluators are used, it adds an extra queue "
      "for each of these evaluators that includes only the nodes that "
      "are generated with a preferred operator. "
      "If only one evaluator and no preferred operator evaluator is used, "
      "the search does not use an alternation open list but a "
      "standard open list with only one queue.");
  parser.document_note("Closed nodes", "Closed node are not re-opened");

  parser.add_option<shared_ptr<FrontToFrontOpenListFactory>>(
      "open", "forward open list");
  parser.add_list_option<shared_ptr<FrontToFrontHeuristic>>(
      "preferred", "use preferred operators of these evaluators", "[]");
  parser.add_option<int>("boost",
                         "boost value for preferred operator open lists", "0");

  regression_eager_search::add_options_to_parser(parser);
  Options opts = parser.parse();

  shared_ptr<regression_eager_search::RegressionEagerSearch> engine;
  if (!parser.dry_run()) {
    opts.set("reopen_closed", false);
    shared_ptr<Evaluator> evaluator = nullptr;
    opts.set("f_eval", evaluator);
    engine = make_shared<regression_eager_search::RegressionEagerSearch>(opts);
  }
  return engine;
}

static Plugin<SearchEngine> _plugin("regression_eager_greedy", _parse);
}  // namespace plugin_regression_eager_greedy
