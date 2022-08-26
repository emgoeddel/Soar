#ifdef ENABLE_ROS

#include "length.h"
#include "objective_table.h"
#include "motor_state.h"
#include "motor/motor.h"

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

/////////////////////////////// LES //////////////////////////////////////

ee_length_objective::ee_length_objective(Symbol* cmd_rt,
                                         soar_interface* si,
                                         motor_state* ms,
                                         objective_input* oi) : objective(cmd_rt,
                                                                          si,
                                                                          ms,
                                                                          oi),
                                                                mtr(ms->get_motor()) {}

bool ee_length_objective::evaluate() {
    std::map<int, trajectory>::iterator i = trajectories.begin();
    for(; i != trajectories.end(); i++) {
        double ee_sum = 0;

        // Get first waypoint ee xyz
        std::map<std::string, double> init_state;
        std::vector<std::string>::iterator n = i->second.joints.begin();
        int m = 0;
        for (; n != i->second.joints.end(); n++) {
            init_state[*n] = i->second.waypoints[0][m];
            m++;
        }
        std::map<std::string, double>::iterator f = i->second.fixed_joints.begin();
        for (; f != i->second.fixed_joints.end(); f++) {
            init_state[f->first] = f->second;
        }
        transform3 ee = mtr->get_ee_frame_transform_at(init_state);
        vec3 prev_xyz;
        ee.position(prev_xyz);

        // Go through rest of waypoints
        for (int w = 1; w < i->second.length; w++) {
            std::map<std::string, double> jnt_state;
            n = i->second.joints.begin();
            m = 0;
            for (; n != i->second.joints.end(); n++) {
                jnt_state[*n] = i->second.waypoints[w][m];
                m++;
            }
            f = i->second.fixed_joints.begin();
            for (; f != i->second.fixed_joints.end(); f++) {
                jnt_state[f->first] = f->second;
            }
            transform3 ee = mtr->get_ee_frame_transform_at(jnt_state);
            vec3 cur_xyz;
            ee.position(cur_xyz);

            // Add straight line dist from prev xyz to total
            ee_sum += sqrt(pow(cur_xyz.x() - prev_xyz.x(), 2) +
                           pow(cur_xyz.y() - prev_xyz.y(), 2) +
                           pow(cur_xyz.z() - prev_xyz.z(), 2));
            prev_xyz = cur_xyz;
        }

        values[i->first] = ee_sum;
        std::cout << "Trajectory " << i->first << ": " << values[i->first] << std::endl;
     }
    return true;
}

objective* make_ee_length_objective(Symbol* cmd_rt,
                                         soar_interface* si,
                                         motor_state* ms,
                                         objective_input* oi) {
    return new ee_length_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* ee_length_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "end-effector-length";
    e->description = "Sum of end-effector movement";
    e->parameters["set-id"] = "Trajectory set";
    e->create = &make_ee_length_objective;
    return e;
}

#endif
