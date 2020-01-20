#include "../search_engines/search_common.h"
#include "bidirectional_eager_search.h"

#include "../front_to_front/front_to_front_heuristic.h"
#include "../front_to_front/front_to_front_open_list_factory.h"
#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace plugin_bidirectional_eager {
static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
  parser.document_synopsis("Regression front to front eager best-first search",
                           "");

  parser.add_option<shared_ptr<FrontToFrontOpenListFactory>>(
      "open_f", "forward open list");
  parser.add_option<shared_ptr<FrontToFrontOpenListFactory>>(
      "open_b", "backward open list");
  parser.add_option<bool>("reopen_closed", "reopen closed nodes", "false");
  parser.add_option<bool>("bdd", "use BDD for duplicate detection", "false");
  parser.add_option<bool>("reeval", "do re evaluation", "false");
  parser.add_option<bool>("front_to_front", "f2f", "false");
  parser.add_option<bool>("use_bgg", "use BGGs", "false");
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
  parser.add_option<bool>(
      "prune_goal", "prune goal state other than the original goal", "false");
  parser.add_option<shared_ptr<FrontToFrontHeuristic>>(
      "bgg_eval", "eval for BGGs", "front_to_front_hmax");

  bidirectional_eager_search::add_options_to_parser(parser);
  Options opts = parser.parse();

  shared_ptr<bidirectional_eager_search::BidirectionalEagerSearch> engine;
  if (!parser.dry_run()) {
    engine =
        make_shared<bidirectional_eager_search::BidirectionalEagerSearch>(opts);
  }

  return engine;
}

static Plugin<SearchEngine> _plugin("bidirectional_eager_greedy", _parse);
}  // namespace plugin_bidirectional_eager
