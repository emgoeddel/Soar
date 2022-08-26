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
    bool evaluate();
};

// AET - Action Execution Time [minimize]
// How long a trajectory will take to execute in seconds
class execution_time_objective : public objective {
public:
    execution_time_objective(Symbol* cmd_rt,
                             soar_interface* si,
                             motor_state* ms,
                             objective_input* oi);
    bool evaluate();
};

#endif
#endif
