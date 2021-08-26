#ifdef ENABLE_ROS

#include "length.h"
#include "objective_table.h"

state_count_objective::state_count_objective(Symbol* cmd_rt,
                                             soar_interface* si,
                                             motor_state* ms,
                                             objective_input* oi) : objective(cmd_rt,
                                                                              si,
                                                                              ms,
                                                                              oi) {}

void state_count_objective::evaluate() {
}

objective* make_state_count_objective(Symbol* cmd_rt,
                                      soar_interface* si,
                                      motor_state* ms,
                                      objective_input* oi) {
    return new state_count_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* state_count_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "state-count";
    e->description = "Number of waypoints in the trajectory";
    e->parameters["set-id"] = "Trajectory set";
    e->create = &make_state_count_objective;
    return e;
}

#endif
