#ifdef ENABLE_ROS

#include "clearance.h"
#include "objective_table.h"
#include "motor_state.h"

min_clearance_objective::min_clearance_objective(Symbol* cmd_rt,
                                                 soar_interface* si,
                                                 motor_state* ms,
                                                 objective_input* oi) : objective(cmd_rt,
                                                                                  si,
                                                                                  ms,
                                                                                  oi) {
    //cc = new collision_checker();
}

min_clearance_objective::~min_clearance_objective() {
    delete cc;
}

bool min_clearance_objective::evaluate() {
    return true;
}

objective* make_min_clearance_objective(Symbol* cmd_rt,
                                        soar_interface* si,
                                        motor_state* ms,
                                        objective_input* oi) {
    return new min_clearance_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* min_clearance_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "min_clearance";
    e->description = "Minimum distance to obstacle across trajectory";
    e->parameters["set-id"] = "Trajectory set";
    e->create = &make_min_clearance_objective;
    return e;
}


#endif
