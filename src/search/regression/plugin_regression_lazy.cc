#include "../front_to_front/front_to_front_open_list_factory.h"
#include "../search_engines/search_common.h"
#include "regression_lazy_search.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace plugin_regression_lazy {
static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
  parser.document_synopsis("Lazy best-first search", "");
  parser.add_option<shared_ptr<FrontToFrontOpenListFactory>>("open",
                                                             "open list");
  parser.add_option<bool>("reopen_closed", "reopen closed nodes", "false");
  parser.add_list_option<shared_ptr<Evaluator>>(
      "preferred", "use preferred operators of these evaluators", "[]");
  SearchEngine::add_succ_order_options(parser);
  SearchEngine::add_options_to_parser(parser);
  Options opts = parser.parse();

  shared_ptr<regression_lazy_search::RegressionLazySearch> engine;
  if (!parser.dry_run()) {
    engine = make_shared<regression_lazy_search::RegressionLazySearch>(opts);
    /*
      TODO: The following two lines look fishy. If they serve a
      purpose, shouldn't the constructor take care of this?
    */
    vector<shared_ptr<Evaluator>> preferred_list =
        opts.get_list<shared_ptr<Evaluator>>("preferred");
    engine->set_preferred_operator_evaluators(preferred_list);
  }

  return engine;
}
static Plugin<SearchEngine> _plugin("regression_lazy", _parse);
}  // namespace plugin_regression_lazy
