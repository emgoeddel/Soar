#ifndef VISIBILITY_H
#define VISIBILITY_H

#ifdef ENABLE_ROS

#include "objective.h"
#include "sgnode_algs.h"

/*
 * VISIBILITY OBJECTIVES
 *
 * Provide the agent with information about how much the arm blocks visibility of
 * object(s) in the environment when moving through a trajectory. Require access to the
 * scene graph (via the motor_state).
 *
 */

// AOO - Average Occlusion of Object [minimize]
// Average amount of occlusion of an object of interest across trajectory
class average_occlusion_objective : public objective {
public:
    average_occlusion_objective(Symbol* cmd_rt,
                                soar_interface* si,
                                motor_state* ms,
                                objective_input* oi);

    double evaluate_on(trajectory& t);

private:
    std::string obj_name;
    sgnode* obj_int;
    sgnode* eye;
    std::vector<const sgnode*> occluders;
    std::vector<view_line> views;
};

// OTO - Occlusion Time of Object [minimize]
// Amount of time that an object is at least partially occluded by the arm
// class occlusion_time_objective : public objective {
// public:
//     occlusion_time_objective(Symbol* cmd_rt,
//                              soar_interface* si,
//                              motor_state* ms,
//                              objective_input* oi);
//     ~occlusion_time_objective();

//     double evaluate_on(trajectory& t);

// private:
//     std::string oi_name;
//     sgnode* oi;
//     sgnode* eye;
// };

#endif
#endif
