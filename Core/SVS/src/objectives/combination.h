#ifndef COMBO_H
#define COMBO_H

#ifdef ENABLE_ROS

#include "objective.h"
#include "length.h"
#include "clearance.h"

/*
 * COMBINATION OBJECTIVES
 *
 * Linear combinations of two other objectives. Ideally this would take ANY two other
 * objectives, but due to Soar interface limitations, I've pre-selected the combinations
 * and hard-coded them into their own objectives.
 *
 * Both of these are versions of CRL as defined in thesis
 */

// CRL (1) - Combined End Effector Rotational and Linear Lengths [minimize]
class ee_combo_objective : public objective {
public:
    ee_combo_objective(Symbol* cmd_rt,
                       soar_interface* si,
                       motor_state* ms,
                       objective_input* oi);
    double evaluate_on(trajectory& t);

private:
    ee_length_objective len;
    ee_rotation_objective rot;

    double LENGTH_WEIGHT;
    double ROTATION_WEIGHT;
};

// CRL (2) - Combined Execution Time and Min Clearance [minimize]
class time_clear_combo_objective : public objective {
public:
    time_clear_combo_objective(Symbol* cmd_rt,
                               soar_interface* si,
                               motor_state* ms,
                               objective_input* oi);
    double evaluate_on(trajectory& t);

private:
    execution_time_objective time;
    min_clearance_objective clear;

    double TIME_WEIGHT;
    double CLEAR_WEIGHT;
};


#endif
#endif
