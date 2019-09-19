#include "../search_engines/search_common.h"
#include "backward_eager_search.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace plugin_backward_eager_greedy {
static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
  parser.document_synopsis("Backward greedy search (backward_eager)", "");
  parser.document_note(
      "Open list",
      "In most cases, backward_eager greedy best first search uses "
      "an alternation open list with one queue for each evaluator. "
      "If preferred operator evaluators are used, it adds an extra queue "
      "for each of these evaluators that includes only the nodes that "
      "are generated with a preferred operator. "
      "If only one evaluator and no preferred operator evaluator is used, "
      "the search does not use an alternation open list but a "
      "standard open list with only one queue.");
  parser.document_note("Closed nodes", "Closed node are not re-opened");
  parser.document_note(
      "Equivalent statements using general backward_eager search",
      "\n```\n--evaluator h2=eval2\n"
      "--search backward_eager_greedy([eval1, h2], preferred=h2, "
      "boost=100)\n```\n"
      "is equivalent to\n"
      "```\n--evaluator h1=eval1 --heuristic h2=eval2\n"
      "--search backward_eager(alt([single(h1), single(h1, pref_only=true), "
      "single(h2), "
      "\n"
      "                    single(h2, pref_only=true)], boost=100),\n"
      "               preferred=h2)\n```\n"
      "------------------------------------------------------------\n"
      "```\n--search backward_eager_greedy([eval1, eval2])\n```\n"
      "is equivalent to\n"
      "```\n--search backward_eager(alt([single(eval1), single(eval2)]))\n```\n"
      "------------------------------------------------------------\n"
      "```\n--evaluator h1=eval1\n"
      "--search backward_eager_greedy(h1, preferred=h1)\n```\n"
      "is equivalent to\n"
      "```\n--evaluator h1=eval1\n"
      "--search backward_eager(alt([single(h1), single(h1, pref_only=true)]),\n"
      "               preferred=h1)\n```\n"
      "------------------------------------------------------------\n"
      "```\n--search backward_eager_greedy(eval1)\n```\n"
      "is equivalent to\n"
      "```\n--search backward_eager(single(eval1))\n```\n",
      true);

  parser.add_list_option<shared_ptr<Evaluator>>("evals", "evaluators");
  parser.add_list_option<shared_ptr<Evaluator>>(
      "preferred", "use preferred operators of these evaluators", "[]");
  parser.add_option<int>("boost",
                         "boost value for preferred operator open lists", "0");

  backward_eager_search::add_options_to_parser(parser);
  Options opts = parser.parse();
  opts.verify_list_non_empty<shared_ptr<Evaluator>>("evals");

  shared_ptr<backward_eager_search::BackwardEagerSearch> engine;
  if (!parser.dry_run()) {
    opts.set("open", search_common::create_greedy_open_list_factory(opts));
    opts.set("reopen_closed", false);
    shared_ptr<Evaluator> evaluator = nullptr;
    opts.set("f_eval", evaluator);
    engine = make_shared<backward_eager_search::BackwardEagerSearch>(opts);
  }
  return engine;
}

static Plugin<SearchEngine> _plugin("backward_eager_greedy", _parse);
}  // namespace plugin_backward_eager_greedy
