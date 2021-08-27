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
    subset_size = sid_fv->get_value();
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
        int selected = i->first;
        double min_value = i->second; // XXX Does not take subset_size into acct
        for (; i != values.end(); i++) {
            if (i->second < min_value) {
                selected = i->first;
                min_value = i->second;
            }
        }
        i = values.begin();
        for(; i != values.end(); i++) {
            if (i->first == selected) {
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
