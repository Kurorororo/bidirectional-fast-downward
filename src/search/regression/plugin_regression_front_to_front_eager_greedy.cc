#include "../search_engines/search_common.h"
#include "regression_front_to_front_eager_search.h"

#include "../front_to_front/front_to_front_heuristic.h"
#include "../front_to_front/front_to_front_open_list_factory.h"
#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace plugin_regression_front_to_front_eager {
static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
  parser.document_synopsis("Regression front to front eager best-first search",
                           "");

  parser.add_option<shared_ptr<FrontToFrontOpenListFactory>>(
      "open_f", "forward open list");
  parser.add_option<shared_ptr<FrontToFrontOpenListFactory>>(
      "open_b", "backward open list");
  parser.add_option<bool>("reopen_closed", "reopen closed nodes", "false");
  parser.add_option<shared_ptr<Evaluator>>(
      "f_eval_f",
      "set forward evaluator for jump statistics. "
      "(Optional; if no evaluator is used, jump statistics will not be "
      "displayed.)",
      OptionParser::NONE);
  parser.add_option<shared_ptr<Evaluator>>(
      "f_eval_b",
      "set backward evaluator for jump statistics. "
      "(Optional; if no evaluator is used, jump statistics will not be "
      "displayed.)",
      OptionParser::NONE);
  parser.add_list_option<shared_ptr<FrontToFrontHeuristic>>(
      "preferred_f", "use forward preferred operators of these evaluators",
      "[]");
  parser.add_list_option<shared_ptr<FrontToFrontHeuristic>>(
      "preferred_b", "use backward preferred operators of these evaluators",
      "[]");
  parser.add_option<int>(
      "threshold", "threashold to decide direction (when jump=ACTION)", "0");

  vector<string> jump_policies;
  vector<string> jump_policies_doc;
  jump_policies.push_back("INTERLEAVING");
  jump_policies_doc.push_back("interleave forward and regression");
  jump_policies.push_back("ACTION");
  jump_policies_doc.push_back(
      "switch to the other direction when the number of successors in the "
      "direction becomes equal to or more than that in the other direction.");
  parser.add_enum_option("jump", jump_policies, "jump policy", "INTERLEAVING",
                         jump_policies_doc);

  regression_front_to_front_eager_search::add_options_to_parser(parser);
  Options opts = parser.parse();

  shared_ptr<
      regression_front_to_front_eager_search::RegressionFrontToFrontEagerSearch>
      engine;
  if (!parser.dry_run()) {
    engine = make_shared<regression_front_to_front_eager_search::
                             RegressionFrontToFrontEagerSearch>(opts);
  }

  return engine;
}

static Plugin<SearchEngine> _plugin("regression_front_to_front_eager_greedy",
                                    _parse);
}  // namespace plugin_regression_front_to_front_eager
