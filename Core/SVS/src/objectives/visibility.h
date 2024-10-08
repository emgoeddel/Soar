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
 * Note that these say subset, but in implementation, they only can take one object.
 *
 */

// Base class for visibility objectives
class base_vis_objective : public objective {
public:
    base_vis_objective(Symbol* cmd_rt,
                       soar_interface* si,
                       motor_state* ms,
                       objective_input* oi);
    ~base_vis_objective();

protected:
    bool has_valid_obj;
    std::string obj_name;
    sgnode* obj_int;
    sgnode* eye;
    std::vector<view_line> views;
    std::vector<std::string> arm;
};

// AOS - Average Occlusion of Subset [minimize]
// Average amount of occlusion of an object of interest across trajectory
class average_occlusion_objective : public base_vis_objective {
public:
    average_occlusion_objective(Symbol* cmd_rt,
                                soar_interface* si,
                                motor_state* ms,
                                objective_input* oi);

    double evaluate_on(trajectory& t);
};

// PRS - Proportion of Trajectory Subset is Occluded [minimize]
// How much of a trajectory is spent with at least two corners of object occluded
class proportion_occluded_objective : public base_vis_objective {
public:
    proportion_occluded_objective(Symbol* cmd_rt,
                                   soar_interface* si,
                                   motor_state* ms,
                                   objective_input* oi);

    double evaluate_on(trajectory& t);
};

// OTS - Occlusion Time of Subset [minimize]
// Amount of time that at least two corners of object are occluded by the arm
class occlusion_time_objective : public base_vis_objective {
public:
    occlusion_time_objective(Symbol* cmd_rt,
                             soar_interface* si,
                             motor_state* ms,
                             objective_input* oi);

    double evaluate_on(trajectory& t);
};

#endif
#endif
