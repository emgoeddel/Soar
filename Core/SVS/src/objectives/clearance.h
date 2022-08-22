#ifndef CLEARANCE_H
#define CLEARANCE_H

#ifdef ENABLE_ROS

#include "objective.h"
#include "motor/collision.h"

/*
 * CLEARANCE OBJECTIVES
 *
 * Provide the agent with information about how close a trajectory takes the arm
 * to objects in the environment. Require access to the scene graph (via the motor_state)
 * and a collision checker for distance queries.
 *
 */

// Finds the minimum clearance between the arm and object across the trajectory
class min_clearance_objective : public objective {
public:
    min_clearance_objective(Symbol* cmd_rt,
                            soar_interface* si,
                            motor_state* ms,
                            objective_input* oi);
    ~min_clearance_objective();

    bool evaluate();

private:
    collision_checker* cc;
};

#endif
#endif
