#include "../search_engines/search_common.h"
#include "interleaving_eager_search.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace plugin_interleaving_eager {
static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
  parser.document_synopsis("Interleaving eager best-first search", "");

  parser.add_option<shared_ptr<OpenListFactory>>("open_f", "forward open list");
  parser.add_option<shared_ptr<OpenListFactory>>("open_b",
                                                 "backward open list");
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
  parser.add_list_option<shared_ptr<Evaluator>>(
      "preferred_f", "use forward preferred operators of these evaluators",
      "[]");
  parser.add_list_option<shared_ptr<Evaluator>>(
      "preferred_b", "use backward preferred operators of these evaluators",
      "[]");

  interleaving_eager_search::add_options_to_parser(parser);
  Options opts = parser.parse();

  shared_ptr<interleaving_eager_search::InterleavingEagerSearch> engine;
  if (!parser.dry_run()) {
    engine =
        make_shared<interleaving_eager_search::InterleavingEagerSearch>(opts);
  }

  return engine;
}

static Plugin<SearchEngine> _plugin("interleaving_eager", _parse);
}  // namespace plugin_interleaving_eager
