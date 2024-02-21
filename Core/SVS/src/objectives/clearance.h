#ifndef CLEARANCE_H
#define CLEARANCE_H

#ifdef ENABLE_ROS

#include "objective.h"
#include "motor/collision.h"
#include "motor/motor.h"

/*
 * CLEARANCE OBJECTIVES
 *
 * Provide the agent with information about how close a trajectory takes the arm
 * to objects in the environment. Require access to the scene graph (via the motor_state)
 * and a collision checker for distance queries.
 *
 */

// MCA - Minimum Clearance, All obstacles [maximize]
// Finds the minimum clearance between the arm and object across the trajectory
class min_clearance_objective : public objective {
public:
    min_clearance_objective(Symbol* cmd_rt,
                            soar_interface* si,
                            motor_state* ms,
                            objective_input* oi);
    ~min_clearance_objective();

    double evaluate_on(trajectory& t);

private:
    std::shared_ptr<motor> mtr;
    std::vector<obstacle> obstacles;
};

// MCS - Minimum Clearance, Subset [maximize]
// Finds the minimum clearance between the arm and specific objects
class min_clear_subset_objective : public objective {
public:
    min_clear_subset_objective(Symbol* cmd_rt,
                               soar_interface* si,
                               motor_state* ms,
                               objective_input* oi);
    ~min_clear_subset_objective();

    double evaluate_on(trajectory& t);

private:
    std::shared_ptr<motor> mtr;
    std::vector<obstacle> subset;
    bool subset_empty;
};

// WAC - Weighted Average Clearance [minimize]
// Averages a cost that increases with lower clearace
class weighted_avg_clearance_objective : public objective {
public:
    weighted_avg_clearance_objective(Symbol* cmd_rt,
                                     soar_interface* si,
                                     motor_state* ms,
                                     objective_input* oi);
    ~weighted_avg_clearance_objective();

    double evaluate_on(trajectory& t);

private:
    std::shared_ptr<motor> mtr;
    std::vector<obstacle> obstacles;
    double MAX_CLR_FOR_AVG;
};

// CHO - Minimum Clearance of Held Object [maximize]
// Finds the minimum clearance between the held object and environment
class held_clearance_objective : public objective {
public:
    held_clearance_objective(Symbol* cmd_rt,
                             soar_interface* si,
                             motor_state* ms,
                             objective_input* oi);
    ~held_clearance_objective();

    double evaluate_on(trajectory& t);

private:
    std::shared_ptr<motor> mtr;
    std::vector<obstacle> obstacles;
};

// CEE - Minimum Clearance of End Effector [maximize]
// Finds the minimum clearance between the end effector links and environment
class ee_clearance_objective : public objective {
public:
    ee_clearance_objective(Symbol* cmd_rt,
                           soar_interface* si,
                           motor_state* ms,
                           objective_input* oi);
    ~ee_clearance_objective();

    double evaluate_on(trajectory& t);

private:
    std::shared_ptr<motor> mtr;
    std::vector<obstacle> obstacles;
};

#endif
#endif
