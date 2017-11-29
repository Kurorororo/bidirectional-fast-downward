#include "root_task.h"

#include "../globals.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state_registry.h"

#include "../utils/collections.h"
#include "../utils/timer.h"

#include <algorithm>
#include <cassert>
#include <memory>
#include <unordered_set>

using namespace std;
using utils::ExitCode;

namespace tasks {
static const int PRE_FILE_VERSION = 3;

#ifndef NDEBUG
static bool check_fact(const FactPair &fact) {
    /*
      We don't put this check into the FactPair ctor to allow (-1, -1)
      facts. Here, we want to make sure that the fact is valid.
    */
    return fact.var >= 0 && fact.value >= 0;
}
#endif

vector<FactPair> read_facts(istream &in) {
    int count;
    in >> count;
    vector<FactPair> conditions;
    conditions.reserve(count);
    for (int i = 0; i < count; ++i) {
        FactPair condition = FactPair::no_fact;
        in >> condition.var >> condition.value;
        check_fact(condition);
        conditions.push_back(condition);
    }
    return conditions;
}

ExplicitVariable::ExplicitVariable(istream &in) {
    check_magic(in, "begin_variable");
    in >> name;
    in >> axiom_layer;
    in >> domain_size;
    in >> ws;
    fact_names.resize(domain_size);
    for (size_t j = 0; j < fact_names.size(); ++j)
        getline(in, fact_names[j]);
    check_magic(in, "end_variable");
}


ExplicitEffect::ExplicitEffect(
    int var, int value, vector<FactPair> &&conditions)
    : fact(var, value), conditions(move(conditions)) {
    assert(check_fact(FactPair(var, value)));
    assert(all_of(this->conditions.begin(), this->conditions.end(), check_fact));
}


void ExplicitOperator::read_pre_post(istream &in) {
    int count;
    in >> count;
    effects.reserve(count);
    for (int i = 0; i < count; ++i) {
        vector<FactPair> conditions = read_facts(in);
        int var, value_pre, value_post;
        in >> var >> value_pre >> value_post;

        if (value_pre != -1) {
            FactPair pre(var, value_pre);
            check_fact(pre);
            preconditions.push_back(pre);
        }
        effects.emplace_back(var, value_post, move(conditions));
    }
}

ExplicitOperator::ExplicitOperator(istream &in, bool is_an_axiom)
    : is_an_axiom(is_an_axiom) {
    if (!is_an_axiom) {
        check_magic(in, "begin_operator");
        in >> ws;
        getline(in, name);
        preconditions = read_facts(in);
        read_pre_post(in);

        int op_cost;
        in >> op_cost;
        cost = g_use_metric ? op_cost : 1;

        g_min_action_cost = min(g_min_action_cost, cost);
        g_max_action_cost = max(g_max_action_cost, cost);

        check_magic(in, "end_operator");
    } else {
        name = "<axiom>";
        cost = 0;
        check_magic(in, "begin_rule");
        read_pre_post(in);
        check_magic(in, "end_rule");
    }
    assert(cost >= 0);
}

void read_and_verify_version(istream &in) {
    int version;
    check_magic(in, "begin_version");
    in >> version;
    check_magic(in, "end_version");
    if (version != PRE_FILE_VERSION) {
        cerr << "Expected preprocessor file version " << PRE_FILE_VERSION
             << ", got " << version << "." << endl;
        cerr << "Exiting." << endl;
        utils::exit_with(ExitCode::INPUT_ERROR);
    }
}

void read_metric(istream &in) {
    check_magic(in, "begin_metric");
    in >> g_use_metric;
    check_magic(in, "end_metric");
}

vector<ExplicitVariable> read_variables(istream &in) {
    int count;
    in >> count;
    vector<ExplicitVariable> variables;
    variables.reserve(count);
    for (int i = 0; i < count; ++i) {
        variables.emplace_back(in);
    }
    return variables;
}

vector<vector<set<FactPair>>> read_mutexes(istream &in, const vector<ExplicitVariable> &variables) {
    vector<vector<set<FactPair>>> inconsistent_facts(variables.size());
    for (size_t i = 0; i < variables.size(); ++i)
        inconsistent_facts[i].resize(variables[i].domain_size);

    int num_mutex_groups;
    in >> num_mutex_groups;

    /*
      NOTE: Mutex groups can overlap, in which case the same mutex
      should not be represented multiple times. The current
      representation takes care of that automatically by using sets.
      If we ever change this representation, this is something to be
      aware of.
    */
    for (int i = 0; i < num_mutex_groups; ++i) {
        check_magic(in, "begin_mutex_group");
        int num_facts;
        in >> num_facts;
        vector<FactPair> invariant_group;
        invariant_group.reserve(num_facts);
        for (int j = 0; j < num_facts; ++j) {
            int var;
            int value;
            in >> var >> value;
            invariant_group.emplace_back(var, value);
        }
        check_magic(in, "end_mutex_group");
        for (const FactPair &fact1 : invariant_group) {
            for (const FactPair &fact2 : invariant_group) {
                if (fact1.var != fact2.var) {
                    /* The "different variable" test makes sure we
                       don't mark a fact as mutex with itself
                       (important for correctness) and don't include
                       redundant mutexes (important to conserve
                       memory). Note that the translator (at least
                       with default settings) removes mutex groups
                       that contain *only* redundant mutexes, but it
                       can of course generate mutex groups which lead
                       to *some* redundant mutexes, where some but not
                       all facts talk about the same variable. */
                    inconsistent_facts[fact1.var][fact1.value].insert(fact2);
                }
            }
        }
    }
    return inconsistent_facts;
}

vector<FactPair> read_goal(istream &in) {
    check_magic(in, "begin_goal");
    vector<FactPair> goals = read_facts(in);
    check_magic(in, "end_goal");
    if (goals.empty()) {
        cerr << "Task has no goal condition!" << endl;
        utils::exit_with(ExitCode::INPUT_ERROR);
    }
    return goals;
}

vector<ExplicitOperator> read_operators(istream &in) {
    int count;
    in >> count;
    vector<ExplicitOperator> operators;
    operators.reserve(count);
    for (int i = 0; i < count; ++i) {
        operators.emplace_back(in, false);
    }
    return operators;
}

vector<ExplicitOperator> read_axioms(istream &in) {
    int count;
    in >> count;
    vector<ExplicitOperator> axioms;
    axioms.reserve(count);
    for (int i = 0; i < count; ++i) {
        axioms.emplace_back(in, true);
    }
    return axioms;
}


RootTask::RootTask(std::istream &in) {
    read_and_verify_version(in);
    read_metric(in);
    variables = read_variables(in);
    mutexes = read_mutexes(in, variables);


    initial_state_values.resize(variables.size());
    check_magic(in, "begin_state");
    for (size_t i = 0; i < variables.size(); ++i) {
        in >> initial_state_values[i];
    }
    check_magic(in, "end_state");
    // TODO: this global variable should disappear eventually.
    g_initial_state_data = initial_state_values;

    for (size_t i = 0; i < variables.size(); ++i) {
        variables[i].axiom_default_value = initial_state_values[i];
    }

    goals = read_goal(in);
    operators = read_operators(in);
    axioms = read_axioms(in);
    /* TODO: We should be stricter here and verify that we
       have reached the end of "in". */
}

const ExplicitVariable &RootTask::get_variable(int var) const {
    assert(utils::in_bounds(var, variables));
    return variables[var];
}

const ExplicitEffect &RootTask::get_effect(
    int op_id, int effect_id, bool is_axiom) const {
    const ExplicitOperator &op = get_operator_or_axiom(op_id, is_axiom);
    assert(utils::in_bounds(effect_id, op.effects));
    return op.effects[effect_id];
}

const ExplicitOperator &RootTask::get_operator_or_axiom(
    int index, bool is_axiom) const {
    if (is_axiom) {
        assert(utils::in_bounds(index, axioms));
        return axioms[index];
    } else {
        assert(utils::in_bounds(index, operators));
        return operators[index];
    }
}

bool RootTask::run_sanity_check() const {
    assert(initial_state_values.size() == variables.size());

    function<bool(const ExplicitOperator &op)> is_axiom =
        [](const ExplicitOperator &op) {
            return op.is_an_axiom;
        };
    assert(none_of(operators.begin(), operators.end(), is_axiom));
    assert(all_of(axioms.begin(), axioms.end(), is_axiom));

    // Check that each variable occurs at most once in the goal.
    unordered_set<int> goal_vars;
    for (const FactPair &goal: goals) {
        goal_vars.insert(goal.var);
    }
    assert(goal_vars.size() == goals.size());
    return true;
}

void RootTask::evaluate_axioms_on_initial_state() const {
    if (!axioms.empty()) {
        // HACK this should not have to go through a state registry.
        // HACK on top of the HACK above: this should not use globals.
        StateRegistry state_registry(
            *this, *g_state_packer, *g_axiom_evaluator, initial_state_values);
        initial_state_values = state_registry.get_initial_state().get_values();
    }
    evaluated_axioms_on_initial_state = true;
}

int RootTask::get_num_variables() const {
    return variables.size();
}

string RootTask::get_variable_name(int var) const {
    return get_variable(var).name;
}

int RootTask::get_variable_domain_size(int var) const {
    return get_variable(var).domain_size;
}

int RootTask::get_variable_axiom_layer(int var) const {
    return get_variable(var).axiom_layer;
}

int RootTask::get_variable_default_axiom_value(int var) const {
    return get_variable(var).axiom_default_value;
}

string RootTask::get_fact_name(const FactPair &fact) const {
    assert(utils::in_bounds(fact.value, get_variable(fact.var).fact_names));
    return get_variable(fact.var).fact_names[fact.value];
}

bool RootTask::are_facts_mutex(const FactPair &fact1, const FactPair &fact2) const {
    if (fact1.var == fact2.var) {
        // Same variable: mutex iff different value.
        return fact1.value != fact2.value;
    }
    assert(utils::in_bounds(fact1.var, mutexes));
    assert(utils::in_bounds(fact1.value, mutexes[fact1.var]));
    return bool(mutexes[fact1.var][fact1.value].count(fact2));
}

int RootTask::get_operator_cost(int index, bool is_axiom) const {
    return get_operator_or_axiom(index, is_axiom).cost;
}

string RootTask::get_operator_name(int index, bool is_axiom) const {
    return get_operator_or_axiom(index, is_axiom).name;
}

int RootTask::get_num_operators() const {
    return operators.size();
}

int RootTask::get_num_operator_preconditions(int index, bool is_axiom) const {
    return get_operator_or_axiom(index, is_axiom).preconditions.size();
}

FactPair RootTask::get_operator_precondition(
    int op_index, int fact_index, bool is_axiom) const {
    const ExplicitOperator &op = get_operator_or_axiom(op_index, is_axiom);
    assert(utils::in_bounds(fact_index, op.preconditions));
    return op.preconditions[fact_index];
}

int RootTask::get_num_operator_effects(int op_index, bool is_axiom) const {
    return get_operator_or_axiom(op_index, is_axiom).effects.size();
}

int RootTask::get_num_operator_effect_conditions(
    int op_index, int eff_index, bool is_axiom) const {
    return get_effect(op_index, eff_index, is_axiom).conditions.size();
}

FactPair RootTask::get_operator_effect_condition(
    int op_index, int eff_index, int cond_index, bool is_axiom) const {
    const ExplicitEffect &effect = get_effect(op_index, eff_index, is_axiom);
    assert(utils::in_bounds(cond_index, effect.conditions));
    return effect.conditions[cond_index];
}

FactPair RootTask::get_operator_effect(
    int op_index, int eff_index, bool is_axiom) const {
    return get_effect(op_index, eff_index, is_axiom).fact;
}

OperatorID RootTask::get_global_operator_id(OperatorID id) const {
    return id;
}

int RootTask::get_num_axioms() const {
    return axioms.size();
}

int RootTask::get_num_goals() const {
    return goals.size();
}

FactPair RootTask::get_goal_fact(int index) const {
    assert(utils::in_bounds(index, goals));
    return goals[index];
}

vector<int> RootTask::get_initial_state_values() const {
    if (!evaluated_axioms_on_initial_state) {
        evaluate_axioms_on_initial_state();
    }
    return initial_state_values;
}

void RootTask::convert_state_values(
    vector<int> &, const AbstractTask *ancestor_task) const {
    if (this != ancestor_task) {
        ABORT("Invalid state conversion");
    }
}

static shared_ptr<AbstractTask> _parse(OptionParser &parser) {
    if (parser.dry_run())
        return nullptr;
    else
        return g_root_task;
}

static PluginShared<AbstractTask> _plugin("no_transform", _parse);
}
