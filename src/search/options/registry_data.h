#ifndef OPTIONS_REGISTRY_DATA_H
#define OPTIONS_REGISTRY_DATA_H

#include "any.h"
#include "doc_utils.h"

#include <vector>



namespace options {
struct PluginData {
    std::string key;
    Any factory;
    std::string group;
    PluginTypeNameGetter type_name_factory;
    DocFactory doc_factory;
    std::type_index type;

    PluginData(std::string key, Any factory, std::string group,
               PluginTypeNameGetter type_name_factory,
               DocFactory doc_factory, std::type_index type);
};


class RegistryData {
    std::vector<PluginTypeInfo> plugin_types;
    std::vector<PluginGroupInfo> plugin_groups;
    std::vector<PluginData> plugins;

public:
    void insert_plugin_type_data(
        const std::string &type_name, const std::string &documentation,
        std::type_index type);

    void insert_plugin_group_data(
        const std::string &group_id, const std::string &doc_title);

    void insert_plugin_data(
        const std::string &key, const Any &factory, const std::string &group,
        PluginTypeNameGetter type_name_factory, DocFactory doc_factory,
        std::type_index type);

    const std::vector<PluginTypeInfo> &get_plugin_type_data() const;
    const std::vector<PluginGroupInfo> &get_plugin_group_data() const;
    const std::vector<PluginData> &get_plugin_data() const;

    static RegistryData *instance() {
        static RegistryData instance_;
        return &instance_;
    }
};
}

#endif
