#include "front_to_front_open_list_factory.h"

#include "../plugin.h"

using namespace std;

template <>
unique_ptr<FrontToFrontStateOpenList>
FrontToFrontOpenListFactory::create_open_list() {
  return create_state_open_list();
}

template <>
unique_ptr<FrontToFrontEdgeOpenList>
FrontToFrontOpenListFactory::create_open_list() {
  return create_edge_open_list();
}

template <>
unique_ptr<FrontToFrontFrontierOpenList>
FrontToFrontOpenListFactory::create_open_list() {
  return create_frontier_open_list();
}

static PluginTypePlugin<FrontToFrontOpenListFactory> _type_plugin(
    "FrontToFrontOpenList",
    // TODO: Replace empty string by synopsis for the wiki page.
    "");
