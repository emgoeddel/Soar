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

// Provides the number of states in a fully-interpolated trajectory, which is a
// rough comparison of trajectory length.
class state_count_objective : public objective {
public:
    state_count_objective(Symbol* cmd_rt,
                          soar_interface* si,
                          motor_state* ms,
                          objective_input* oi);
    void evaluate();
};

#endif
#endif
