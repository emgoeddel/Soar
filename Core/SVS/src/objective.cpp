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
    filter_val_c<bool>* mx_fv = dynamic_cast<filter_val_c<bool>*>((*input)["maximize"]);
    maximize = mx_fv->get_value();
    filter_val_c<std::string>* nm_fv = dynamic_cast<filter_val_c<std::string>*>((*input)["name"]);
    name = nm_fv->get_value();
    filter_val_c<std::string>* type_fv = dynamic_cast<filter_val_c<std::string>*>((*input)["output-type"]);
    ot = str_to_output(type_fv->get_value());

    if (input->count("subset-type")) {
        filter_val_c<std::string>* st_fv =
            dynamic_cast<filter_val_c<std::string>*>((*input)["subset-type"]);
        subset_type = st_fv->get_value();
    } else subset_type = "exact";

    if (input->count("previous-selection")) {
        filter_val_c<std::string>* prev_fv =
            dynamic_cast<filter_val_c<std::string>*>((*input)["previous-selection"]);
        std::string obj_name = prev_fv->get_value();

        std::stringstream ss;
        ss << obj_name << "-d";
        std::string potential_duplicate = ss.str();

        objective* prev_obj;
        if (ms->has_objective(set_id, potential_duplicate))
            // Base any further reasoning off the SECOND call to a previous objective
            // if it exists
            // XXX Note doesn't support calling the same objective 3x
            prev_obj = ms->get_objective(set_id, potential_duplicate);
        else
            // Otherwise there's only one instance of the previous objective
            prev_obj = ms->get_objective(set_id, obj_name);

        std::map<int, double> prev_outputs = prev_obj->get_outputs();
        std::map<int, double>::iterator p = prev_outputs.begin();
        for (; p != prev_outputs.end(); p++) {
            if (p->second) prev_selected.insert(p->first);
        }
    }
}

objective::~objective() {
    for (objective_input::iterator i = input->begin(); i != input->end(); i++) {
        delete i->second;
    }
    delete input;
}

bool pair_comp_min(std::pair<int, double> a, std::pair<int, double> b) {
    return a.second < b.second;
}

bool pair_comp_max(std::pair<int, double> a, std::pair<int, double> b) {
    return a.second > b.second;
}

bool objective::evaluate() {
    std::cout << "=============== " << name << " ===============" << std::endl;
    std::map<int, trajectory>::iterator i = trajectories.begin();
    for (; i != trajectories.end(); i++) {
        if (!prev_selected.empty() && !prev_selected.count(i->first)) continue;
        values[i->first] = evaluate_on(i->second);
        std::cout << i->first << ": " << values[i->first] << std::endl;
     }

    return true;
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

        if (maximize) {
            std::sort(sorted.begin(), sorted.end(), pair_comp_max);
        } else {
            std::sort(sorted.begin(), sorted.end(), pair_comp_min);
        }

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

        if (maximize) {
            std::sort(sorted.begin(), sorted.end(), pair_comp_max);
        } else {
            std::sort(sorted.begin(), sorted.end(), pair_comp_min);
        }

        int c_i = 0;
        double cutoff_value;
        std::vector<std::pair<int, double> >::iterator c = sorted.begin();
        for (; c != sorted.end(); c++) {
            if (c_i == subset_size-1) {
                cutoff_value = c->second;
                break;
            }
            c_i++;
        }

        int o = 0;
        std::vector<std::pair<int, double> >::iterator j = sorted.begin();
        for (; j != sorted.end(); j++) {
            if (subset_type == "exact") {
                if (o < subset_size) outputs[j->first] = 1;
            } else if (subset_type == "strict") {
                if (o < subset_size && j->second < cutoff_value) outputs[j->first] = 1;
                if (j->second == cutoff_value && outputs[subset_size] > cutoff_value)
                    outputs[j->first] = 1;
            } else if (subset_type == "loose") {
                if (j->second <= cutoff_value) outputs[j->first] = 1;
            } else outputs[j->first] = 0;

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

int objective::get_selected() {
    if (ot == SELECT) {
        std::map<int, double>::iterator o = outputs.begin();
        for (; o != outputs.end(); o++) {
            if (o->second == 1) return o->first;
        }
    }
    else {
        std::cout << "[Error] No selection to report from objective" << std::endl;
    }
    return 0;
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
