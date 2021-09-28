#ifdef ENABLE_ROS

#include "objective.h"
#include "motor_state.h"

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
}

objective::~objective() {
    delete input;
}

// Assumes values have already been computed
void objective::update_outputs() {
    switch (ot) {
    case RANK:
        break;
    case SELECT: {
        std::map<int, double>::iterator i = values.begin();
        std::list<int> sel_inds;
        std::list<double> sel_vals;
        for (; i != values.end(); i++) {
            std::list<double>::iterator j = sel_vals.begin();
            std::list<int>::iterator k = sel_inds.begin();
            for (; j != sel_vals.end(); j++) {
                if (i->second < *j) break;
                k++;
            }
            if (sel_vals.size() < subset_size || j != sel_vals.end()) {
                sel_vals.insert(j, i->second);
                sel_inds.insert(k, i->first);

                if (sel_vals.size() > subset_size) {
                    sel_vals.pop_back();
                    sel_inds.pop_back();
                }
            }
        }

        i = values.begin();
        for(; i != values.end(); i++) {
            std::list<int>::iterator k = sel_inds.begin();
            bool is_selected = false;
            for (; k != sel_inds.end(); k++) {
                if (*k == i->first) {
                    is_selected = true;
                    break;
                }
            }

            if (is_selected) {
                outputs[i->first] = 1;
            } else {
                outputs[i->first] = 0;
            }
        }
    } break;
    case VALUE:
        break;
    default:
        break;
    }
}

void objective::set_status(const std::string& msg) {
    if (status == msg) return;
    status = msg;
    if (status_wme) si->remove_wme(status_wme);
    if (cmd_rt && si)
        status_wme = si->make_wme(cmd_rt, si->get_common_syms().status, status);
}

#endif
