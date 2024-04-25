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

double waypoints_objective::evaluate_on(trajectory& t) {
    return t.length;
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

/////////////////////////////// Planning Time //////////////////////////////////////

planning_time_objective::planning_time_objective(Symbol* cmd_rt,
                                                 soar_interface* si,
                                                 motor_state* ms,
                                                 objective_input* oi) : objective(cmd_rt,
                                                                                  si,
                                                                                  ms,
                                                                                  oi) {}

double planning_time_objective::evaluate_on(trajectory& t) {
    return t.planning_time;
}

objective* make_planning_time_objective(Symbol* cmd_rt,
                                        soar_interface* si,
                                        motor_state* ms,
                                        objective_input* oi) {
    return new planning_time_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* planning_time_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "planning-time";
    e->description = "Amount of time taken to plan the trajectory";
    e->parameters["set-id"] = "Trajectory set";
    e->create = &make_planning_time_objective;
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

double execution_time_objective::evaluate_on(trajectory& t) {
    return t.times[t.length - 1]; // time at last wp
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

double total_joint_objective::evaluate_on(trajectory& t) {
    double j_sum = 0;
    for (int w = 1; w < t.length; w++) {
        for (int j = 0; j < t.waypoints[w].size(); j++) {
            j_sum += fabs(t.waypoints[w][j] - t.waypoints[w-1][j]);
        }
    }
    return j_sum;
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

/////////////////////////////// ELJ //////////////////////////////////////

euclidean_joint_objective::euclidean_joint_objective(Symbol* cmd_rt,
                                                     soar_interface* si,
                                                     motor_state* ms,
                                                     objective_input* oi) : objective(cmd_rt,
                                                                                      si,
                                                                                      ms,
                                                                                      oi) {}

double euclidean_joint_objective::evaluate_on(trajectory& t) {
    double j_sum = 0;
    for (int w = 1; w < t.length; w++) {
        double w_sum = 0;
        for (int j = 0; j < t.waypoints[w].size(); j++) {
            w_sum += powf((t.waypoints[w][j] - t.waypoints[w-1][j]), 2);
        }
        j_sum += sqrt(w_sum);
    }
    return j_sum;
}

objective* make_euclidean_joint_objective(Symbol* cmd_rt,
                                          soar_interface* si,
                                          motor_state* ms,
                                          objective_input* oi) {
    return new euclidean_joint_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* euclidean_joint_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "euclidean-joint";
    e->description = "Sum of straight-line (Euclidean) joint space motion";
    e->parameters["set-id"] = "Trajectory set";
    e->create = &make_euclidean_joint_objective;
    return e;
}

/////////////////////////////// SSJ //////////////////////////////////////

sum_square_joint_objective::sum_square_joint_objective(Symbol* cmd_rt,
                                                       soar_interface* si,
                                                       motor_state* ms,
                                                       objective_input* oi) : objective(cmd_rt,
                                                                                        si,
                                                                                        ms,
                                                                                        oi) {}

double sum_square_joint_objective::evaluate_on(trajectory& t) {
    double j_sum = 0;
    for (int w = 1; w < t.length; w++) {
        for (int j = 0; j < t.waypoints[w].size(); j++) {
            j_sum += powf((t.waypoints[w][j] - t.waypoints[w-1][j]), 2);
        }
    }
    return j_sum;
}

objective* make_sum_square_joint_objective(Symbol* cmd_rt,
                                           soar_interface* si,
                                           motor_state* ms,
                                           objective_input* oi) {
    return new sum_square_joint_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* sum_square_joint_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "sum-square-joint";
    e->description = "Sum of squared joint space motion";
    e->parameters["set-id"] = "Trajectory set";
    e->create = &make_sum_square_joint_objective;
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

double ee_length_objective::evaluate_on(trajectory& t) {
    double ee_sum = 0;

    // Get first waypoint ee xyz
    std::map<std::string, double> init_state;
    std::vector<std::string>::iterator n = t.joints.begin();
    int m = 0;
    for (; n != t.joints.end(); n++) {
        init_state[*n] = t.waypoints[0][m];
        m++;
    }
    std::map<std::string, double>::iterator f = t.fixed_joints.begin();
    for (; f != t.fixed_joints.end(); f++) {
        init_state[f->first] = f->second;
    }
    transform3 ee = mtr->get_ee_frame_transform_at(init_state);
    vec3 prev_xyz;
    ee.position(prev_xyz);

    // Go through rest of waypoints
    for (int w = 1; w < t.length; w++) {
        std::map<std::string, double> jnt_state;
        n = t.joints.begin();
        m = 0;
        for (; n != t.joints.end(); n++) {
            jnt_state[*n] = t.waypoints[w][m];
            m++;
        }
        f = t.fixed_joints.begin();
        for (; f != t.fixed_joints.end(); f++) {
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

    return ee_sum;
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
    e->description = "Sum of end-effector linear movement";
    e->parameters["set-id"] = "Trajectory set";
    e->create = &make_ee_length_objective;
    return e;
}

/////////////////////////////// LER //////////////////////////////////////

ee_rotation_objective::ee_rotation_objective(Symbol* cmd_rt,
                                             soar_interface* si,
                                             motor_state* ms,
                                             objective_input* oi) : objective(cmd_rt,
                                                                              si,
                                                                              ms,
                                                                              oi),
                                                                    mtr(ms->get_motor()) {}

double ee_rotation_objective::evaluate_on(trajectory& t) {
    double ee_sum = 0;

    // Get first waypoint ee xform
    std::map<std::string, double> init_state;
    std::vector<std::string>::iterator n = t.joints.begin();
    int m = 0;
    for (; n != t.joints.end(); n++) {
        init_state[*n] = t.waypoints[0][m];
        m++;
    }
    std::map<std::string, double>::iterator f = t.fixed_joints.begin();
    for (; f != t.fixed_joints.end(); f++) {
        init_state[f->first] = f->second;
    }
    transform3 prev_xform = mtr->get_ee_frame_transform_at(init_state);

    // Go through rest of waypoints
    for (int w = 1; w < t.length; w++) {
        std::map<std::string, double> jnt_state;
        n = t.joints.begin();
        m = 0;
        for (; n != t.joints.end(); n++) {
            jnt_state[*n] = t.waypoints[w][m];
            m++;
        }
        f = t.fixed_joints.begin();
        for (; f != t.fixed_joints.end(); f++) {
            jnt_state[f->first] = f->second;
        }
        transform3 cur_xform = mtr->get_ee_frame_transform_at(jnt_state);

        // Add angular dist from prev to total
        ee_sum += fabs(cur_xform.angle_difference(prev_xform));
        prev_xform = cur_xform;
    }

    return ee_sum;
}

objective* make_ee_rotation_objective(Symbol* cmd_rt,
                                         soar_interface* si,
                                         motor_state* ms,
                                         objective_input* oi) {
    return new ee_rotation_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* ee_rotation_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "end-effector-rotation";
    e->description = "Sum of end-effector rotational movement";
    e->parameters["set-id"] = "Trajectory set";
    e->create = &make_ee_rotation_objective;
    return e;
}

#endif
