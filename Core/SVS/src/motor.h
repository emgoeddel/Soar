#ifndef MOTOR_H
#define MOTOR_H

#ifdef ENABLE_ROS

#include <vector>

#include "robot_model.h"
#include "planning_problem.h"

/*
 * motor class
 *
 * Provides an interface to motion planning and robot kinematics
 *
 */

class motor {
public:
    motor(std::string urdf);
    ~motor();

    robot_model* get_model_ptr();
    std::vector<std::string> get_link_names();

    std::string robot_name() { return model.name; }

    bool new_planner_query(int id, motor_query q, motor_state* msp);

private:
    robot_model model;
    std::vector<planning_problem*> ongoing;
};

#endif
#endif
