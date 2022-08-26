#ifdef ENABLE_ROS

#include "length.h"
#include "objective_table.h"
#include "motor_state.h"

/////////////////////////////// Waypoints //////////////////////////////////////

waypoints_objective::waypoints_objective(Symbol* cmd_rt,
                                         soar_interface* si,
                                         motor_state* ms,
                                         objective_input* oi) : objective(cmd_rt,
                                                                          si,
                                                                          ms,
                                                                          oi) {}

bool waypoints_objective::evaluate() {
    std::map<int, trajectory>::iterator i = trajectories.begin();
    for(; i != trajectories.end(); i++) {
        values[i->first] = i->second.length;
        //std::cout << "Trajectory " << i->first << ": " << values[i->first] << std::endl;
     }
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

/////////////////////////////// AET //////////////////////////////////////

execution_time_objective::execution_time_objective(Symbol* cmd_rt,
                                                   soar_interface* si,
                                                   motor_state* ms,
                                                   objective_input* oi) : objective(cmd_rt,
                                                                                    si,
                                                                                    ms,
                                                                                    oi) {}

bool execution_time_objective::evaluate() {
    std::map<int, trajectory>::iterator i = trajectories.begin();
    for(; i != trajectories.end(); i++) {
        values[i->first] = i->second.times[i->second.length - 1]; // time at last wp
        std::cout << "Trajectory " << i->first << ": " << values[i->first] << std::endl;
     }
    return true;
}

objective* make_execution_time_objective(Symbol* cmd_rt,
                                         soar_interface* si,
                                         motor_state* ms,
                                         objective_input* oi) {
    return new execution_time_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* execution_time_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "execution-time";
    e->description = "Amount of time to execute";
    e->parameters["set-id"] = "Trajectory set";
    e->create = &make_execution_time_objective;
    return e;
}

/////////////////////////////// TJM //////////////////////////////////////

total_joint_objective::total_joint_objective(Symbol* cmd_rt,
                                                   soar_interface* si,
                                                   motor_state* ms,
                                                   objective_input* oi) : objective(cmd_rt,
                                                                                    si,
                                                                                    ms,
                                                                                    oi) {}

bool total_joint_objective::evaluate() {
    std::map<int, trajectory>::iterator i = trajectories.begin();
    for(; i != trajectories.end(); i++) {
        double j_sum = 0;
        for (int w = 1; w < i->second.length; w++) {
            for (int j = 0; j < i->second.waypoints[w].size(); j++) {
                j_sum += fabs(i->second.waypoints[w][j] - i->second.waypoints[w-1][j]);
            }
        }

        values[i->first] = j_sum;
        std::cout << "Trajectory " << i->first << ": " << values[i->first] << std::endl;
     }
    return true;
}

objective* make_total_joint_objective(Symbol* cmd_rt,
                                         soar_interface* si,
                                         motor_state* ms,
                                         objective_input* oi) {
    return new total_joint_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* total_joint_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "total-joint-movement";
    e->description = "Sum of all joint movements";
    e->parameters["set-id"] = "Trajectory set";
    e->create = &make_total_joint_objective;
    return e;
}

#endif
