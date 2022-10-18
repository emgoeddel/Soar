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

// PTO - Proportion of Trajectory spent Over a selected object
// Calculates how much of a trajectory is spent with part of the arm "over" object
class proportion_over_objective : public objective {
public:
    proportion_over_objective(Symbol* cmd_rt,
                              soar_interface* si,
                              motor_state* ms,
                              objective_input* oi);
    ~proportion_over_objective();

    double evaluate_on(trajectory& t);

private:
    bool has_valid_obj;
    std::string obj_name;
    collision_checker* cc;
};

// WPO - Weighted Proportion of trajectory spent Over a selected object
// Calculates how much of a trajectory is spent with part of the arm "over" object,
// with more weight given when more parts of the arm are "over"
// class weighted_prop_over_objective : public objective {
// public:
//     weighted_prop_over_objective(Symbol* cmd_rt,
//                                  soar_interface* si,
//                                  motor_state* ms,
//                                  objective_input* oi);
//     ~weighted_prop_over_objective();

//     double evaluate_on(trajectory& t);

// private:
//     collision_checker* cc;
// };

// TTO - Time during Trajectory spent Over a selected object
// Calculates how much time is spent with part of the arm "over" object
// class time_over_objective : public objective {
// public:
//     time_over_objective(Symbol* cmd_rt,
//                         soar_interface* si,
//                         motor_state* ms,
//                         objective_input* oi);
//     ~time_over_objective();

//     double evaluate_on(trajectory& t);

// private:
//     collision_checker* cc;
// };

#endif
#endif
