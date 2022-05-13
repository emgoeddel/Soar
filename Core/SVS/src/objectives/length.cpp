#ifdef ENABLE_ROS

#include "length.h"
#include "objective_table.h"
#include "motor_state.h"

waypoints_objective::waypoints_objective(Symbol* cmd_rt,
                                         soar_interface* si,
                                         motor_state* ms,
                                         objective_input* oi) : objective(cmd_rt,
                                                                          si,
                                                                          ms,
                                                                          oi) {
    std::cout << "Instantiating the waypoints objective!" << std::endl;
}

bool waypoints_objective::evaluate() {
    values = ms->trajectory_lengths(set_id);
    //std::cout << "Computed the length values for " << values.size()
    //          << " trajectories" << std::endl;
    return true;
}

objective* make_waypoints_objective(Symbol* cmd_rt,
                                    soar_interface* si,
                                    motor_state* ms,
                                    objective_input* oi) {
    return new waypoints_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* waypoints_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "waypoints";
    e->description = "Number of waypoints in the trajectory";
    e->parameters["set-id"] = "Trajectory set";
    e->create = &make_waypoints_objective;
    return e;
}

#endif
