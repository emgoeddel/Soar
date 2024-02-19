#ifndef ENDPOINT_H
#define ENDPOINT_H

#ifdef ENABLE_ROS

#include "objective.h"

class motor;

/*
 * ENDPOINT OBJECTIVES
 *
 * Provide the agent with information about where the endpoint (for the end effector!)
 * of a trajectory is. These really only make sense when using target regions, because
 * for point  targets, all trajectories will have the same endpoint values.
 *
 */

// CTR - Endpoint Centrality [minimize OR maximize]
// Returns how close to the center of a given area the endpoint is
// Scale is 0 (very edge) to 1 (perfectly centered)
class centrality_objective : public objective {
public:
    centrality_objective(Symbol* cmd_rt,
                         soar_interface* si,
                         motor_state* ms,
                         objective_input* oi);
    double evaluate_on(trajectory& t);
private:
    vec3 center_pt;
    double max_dist;
    std::shared_ptr<motor> mtr;
};

// DTO - Endpoint Distance to Object [minimize OR maximize]
// Returns distance of the end effector's end point to a given object
class distance_objective : public objective {
public:
    distance_objective(Symbol* cmd_rt,
                       soar_interface* si,
                       motor_state* ms,
                       objective_input* oi);
    double evaluate_on(trajectory& t);
private:
    bool has_valid_obj;
    obstacle object;
    std::shared_ptr<motor> mtr;
};

#endif
#endif
