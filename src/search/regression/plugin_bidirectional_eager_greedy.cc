#include "../front_to_front/front_to_front_heuristic.h"
#include "../front_to_front/front_to_front_open_list_factory.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../search_engines/search_common.h"
#include "bidirectional_eager_search.h"

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
  parser.add_option<int>("max_steps", "max steps for one direction", "0");
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
  vector<string> d_node_type_names;
  vector<string> d_node_type_docs;
  d_node_type_names.push_back("FRONT_TO_END");
  d_node_type_docs.push_back("use the initial/goal state");
  d_node_type_names.push_back("TTBS");
  d_node_type_docs.push_back("use the top state in the other open");
  d_node_type_names.push_back("BGG");
  d_node_type_docs.push_back("use BGG");
  d_node_type_names.push_back("MAX_G");
  d_node_type_docs.push_back("use state with max g-value");
  parser.add_enum_option("d_node_type", d_node_type_names, "D node type",
                         "FRONT_TO_END", d_node_type_docs);

  vector<string> reeval_names;
  vector<string> reeval_docs;
  reeval_names.push_back("NO");
  reeval_docs.push_back("Do not reeval");
  reeval_names.push_back("NOT_SIMILAR");
  reeval_docs.push_back("Reeval if not similar");
  reeval_names.push_back("ALL");
  reeval_docs.push_back("Reeval all nodes");
  parser.add_enum_option("reeval", reeval_names, "Reevaluation method", "NO",
                         reeval_docs);

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
