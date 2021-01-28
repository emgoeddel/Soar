#ifndef MOTOR_H
#define MOTOR_H

#ifdef ENABLE_ROS

#include <map>
#include <mutex>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "mat.h"
#include "trajectory.h"
#include "robot_model.h"

/*
 * motor class
 *
 * Provides an interface to motion planning and robot kinematics
 *
 */

class motor {
public:
    motor(std::string urdf);

    robot_model* get_model_ptr() { return &model; }
    std::vector<std::string> get_link_names();

    std::string robot_name() { return model.name; }

private:
    robot_model model;

    ompl::geometric::SimpleSetup* ompl_ss;
};

#endif
#endif
