#ifdef ENABLE_ROS

#include "objective.h"
#include "motor_state.h"

#include <algorithm>

objective::objective(Symbol* cmd_rt,
                     soar_interface* si,
                     motor_state* ms,
                     objective_input* oi) : cmd_rt(cmd_rt),
                                            si(si),
                                            ms(ms),
                                            input(oi) {
    filter_val_c<int>* sid_fv = dynamic_cast<filter_val_c<int>*>((*input)["set-id"]);
    set_id = sid_fv->get_value();
    filter_val_c<int>* n_fv = dynamic_cast<filter_val_c<int>*>((*input)["number"]);
    subset_size = n_fv->get_value();
    filter_val_c<std::string>* nm_fv = dynamic_cast<filter_val_c<std::string>*>((*input)["name"]);
    name = nm_fv->get_value();
    filter_val_c<std::string>* type_fv = dynamic_cast<filter_val_c<std::string>*>((*input)["output-type"]);
    ot = str_to_output(type_fv->get_value());
}

objective::~objective() {
    delete input;
}

bool pair_comp(std::pair<int, double> a, std::pair<int, double> b) {
    return a.second < b.second;
}

void objective::get_latest_trajectories() {
    ms->get_latest_trajectories(set_id, trajectories);
}

// Assumes values have already been computed
bool objective::update_outputs() {
    int prev_size = outputs.size();

    switch (ot) {
    case RANK: {
        std::map<int, double>::iterator i = values.begin();
        std::vector<std::pair<int, double> > sorted;
        for (; i != values.end(); i++) {
            sorted.push_back(std::pair<int, double>(i->first, i->second));
        }

        std::sort(sorted.begin(), sorted.end(), pair_comp);

        int rank = 1;
        std::vector<std::pair<int, double> >::iterator j = sorted.begin();
        for (; j != sorted.end(); j++) {
            outputs[j->first] = rank;
            rank++;
        }
    } break;
    case SELECT: {
        std::map<int, double>::iterator i = values.begin();
        std::vector<std::pair<int, double> > sorted;
        for (; i != values.end(); i++) {
            sorted.push_back(std::pair<int, double>(i->first, i->second));
        }

        std::sort(sorted.begin(), sorted.end(), pair_comp);

        int o = 0;
        std::vector<std::pair<int, double> >::iterator j = sorted.begin();
        for (; j != sorted.end(); j++) {
            if (o < subset_size) outputs[j->first] = 1;
            else outputs[j->first] = 0;
            o++;
        }
    } break;
    case VALUE:
    default: {
        std::map<int, double>::iterator i = values.begin();
        for (; i != values.end(); i++) {
            outputs[i->first] = i->second;
        }
    } break;
    }

    if (outputs.size() > prev_size) return true;
    return false;
}

void objective::set_status(const std::string& msg) {
    if (status == msg) return;
    status = msg;
    if (status_wme) si->remove_wme(status_wme);
    if (cmd_rt && si)
        status_wme = si->make_wme(cmd_rt, si->get_common_syms().status, status);
}

OutputType objective::str_to_output(std::string s) {
    if (s == "value") return VALUE;
    if (s == "rank") return RANK;
    if (s == "select") return SELECT;
    else {
        std::cout << "[ERROR] Invalid objective output type " << s << std::endl;
        return SELECT;
    }
}

#endif
