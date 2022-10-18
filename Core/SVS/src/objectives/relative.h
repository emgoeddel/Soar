#ifndef RELATIVE_H
#define RELATIVE_H

#ifdef ENABLE_ROS

#include "objective.h"
#include "motor/collision.h"

/*
 * RELATIVE OBJECTIVES
 *
 * Provide the agent with information about the arm's relationship with other objects in
 * the environment during a trajectory. Require access to the scene graph (via the
 * motor_state) and a collision checker for distance queries.
 *
 * XXX Ideally these would integrate with SVS filters that extract relationship, but that
 *     is a pretty big re-engineering project. Replace "over" with any arm relationship
 *     in the extended case.
 *
 */

// Base class for "over" relationships
class base_over_objective : public objective {
public:
    base_over_objective(Symbol* cmd_rt,
                        soar_interface* si,
                        motor_state* ms,
                        objective_input* oi);
    ~base_over_objective();

protected:
    bool has_valid_obj;
    std::string obj_name;
    collision_checker* cc;
};

// PTO - Proportion of Trajectory spent Over a selected object
// Calculates how much of a trajectory is spent with part of the arm "over" object
class proportion_over_objective : public base_over_objective {
public:
    proportion_over_objective(Symbol* cmd_rt,
                              soar_interface* si,
                              motor_state* ms,
                              objective_input* oi);
    double evaluate_on(trajectory& t);
};

// WPO - Weighted Proportion of trajectory spent Over a selected object
// Calculates how much of a trajectory is spent with part of the arm "over" object,
// with more weight given when more parts of the arm are "over"
// XXX This is not trivial to calculate given current collision capabilities

// class weighted_prop_over_objective : public base_over_objective {
// public:
//     weighted_prop_over_objective(Symbol* cmd_rt,
//                                  soar_interface* si,
//                                  motor_state* ms,
//                                  objective_input* oi);
//     double evaluate_on(trajectory& t);
// };

// TTO - Time during Trajectory spent Over a selected object
// Calculates how much time is spent with part of the arm "over" object
class time_over_objective : public base_over_objective {
public:
    time_over_objective(Symbol* cmd_rt,
                        soar_interface* si,
                        motor_state* ms,
                        objective_input* oi);
    double evaluate_on(trajectory& t);
};

#endif
#endif
