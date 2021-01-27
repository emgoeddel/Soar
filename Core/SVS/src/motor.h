#ifndef MOTOR_H
#define MOTOR_H

#ifdef ENABLE_ROS

#include <map>
#include <mutex>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
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
    motor(ros::NodeHandle& nh);
    std::map<std::string, transform3> get_link_transforms();
    std::map<std::string, transform3> get_link_transforms_at(std::map<std::string, double> p);
    std::vector<std::string> get_link_names();

    void set_joints(std::map<std::string, double>& joints_in, bool verify = false);
    std::map<std::string, double> get_joints();

    std::string robot_name() { return model.name; }

private:
    ros::NodeHandle& n;

    robot_model model;

    std::map<std::string, double> current_joints;
    std::mutex joints_mtx;

    ompl::geometric::SimpleSetup* ompl_ss;
};

#endif
#endif
