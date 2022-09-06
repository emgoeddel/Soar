#ifndef LENGTH_H
#define LENGTH_H

#ifdef ENABLE_ROS

#include "objective.h"

/*
 * LENGTH OBJECTIVES
 *
 * Provide the agent with information about how long the trajectories in a set
 * are, usually wrt space or time, although length can be computed a number of
 * different ways. These objectives take only the trajectory itself as input.
 *
 */

// Waypoints [minimize]
// Provides the number of waypoints in a fully-interpolated trajectory, which is a
// rough comparison of trajectory length.
class waypoints_objective : public objective {
public:
    waypoints_objective(Symbol* cmd_rt,
                        soar_interface* si,
                        motor_state* ms,
                        objective_input* oi);
    double evaluate_on(trajectory& t);
};

// AET - Action Execution Time [minimize]
// How long a trajectory will take to execute in seconds
class execution_time_objective : public objective {
public:
    execution_time_objective(Symbol* cmd_rt,
                             soar_interface* si,
                             motor_state* ms,
                             objective_input* oi);
    double evaluate_on(trajectory& t);
};

// TJM - Total Joint Movement [minimize]
// Total amount that joints move across the trajectory in radians
class total_joint_objective : public objective {
public:
    total_joint_objective(Symbol* cmd_rt,
                          soar_interface* si,
                          motor_state* ms,
                          objective_input* oi);
    double evaluate_on(trajectory& t);
};

class motor;

// LES - Length in End-effector Space [minimize]
// Total amount that the hand moves across the trajectory in m
class ee_length_objective : public objective {
public:
    ee_length_objective(Symbol* cmd_rt,
                        soar_interface* si,
                        motor_state* ms,
                        objective_input* oi);
    double evaluate_on(trajectory& t);

private:
    std::shared_ptr<motor> mtr;
};

#endif
#endif
