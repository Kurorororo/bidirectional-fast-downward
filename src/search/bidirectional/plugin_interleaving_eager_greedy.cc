
#include "../search_engines/search_common.h"
#include "interleaving_eager_search.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace plugin_interleaving_eager_greedy {

static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
  parser.document_synopsis("Interleaving greedy search (eager)", "");
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

  parser.add_list_option<shared_ptr<Evaluator>>("evals_f",
                                                "forward evaluators");
  parser.add_list_option<shared_ptr<Evaluator>>("evals_b",
                                                "backward evaluators");
  parser.add_list_option<shared_ptr<Evaluator>>(
      "preferred_f", "use preferred operators of these forward evaluators",
      "[]");
  parser.add_list_option<shared_ptr<Evaluator>>(
      "preferred_b", "use preferred operators of these backward evaluators",
      "[]");
  parser.add_option<int>("boost",
                         "boost value for preferred operator open lists", "0");

  interleaving_eager_search::add_options_to_parser(parser);
  Options opts = parser.parse();
  opts.verify_list_non_empty<shared_ptr<Evaluator>>("evals_f");
  opts.verify_list_non_empty<shared_ptr<Evaluator>>("evals_b");

  shared_ptr<interleaving_eager_search::InterleavingEagerSearch> engine;
  if (!parser.dry_run()) {
    auto open_pair =
        search_common::create_bidirectional_greedy_open_list_factory(opts);
    opts.set("open_f", open_pair.first);
    opts.set("open_b", open_pair.second);
    opts.set("reopen_closed", false);
    shared_ptr<Evaluator> evaluator = nullptr;
    opts.set("f_eval_f", evaluator);
    opts.set("f_eval_b", evaluator);
    engine =
        make_shared<interleaving_eager_search::InterleavingEagerSearch>(opts);
  }
  return engine;
}

static Plugin<SearchEngine> _plugin("interleaving_eager_greedy", _parse);
}  // namespace plugin_interleaving_eager_greedy
