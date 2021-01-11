#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#ifdef ENABLE_ROS

#include <map>
#include <ros/ros.h>
#include <urdf/model.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "mat.h"

struct joint_value_limit {
    double max;
    double min;
};

struct robot_model {
    robot_model() : name("none") {};

    bool init(std::string robot_desc);

    std::string name;

    // link names
    std::set<std::string> links_of_interest;

    // joint information
    std::set<std::string> joint_names;
    std::map<std::string, joint_value_limit> pose_limits;
    std::map<std::string, joint_value_limit> vel_limits;
    std::map<std::string, joint_value_limit> accel_limits;
};

/*
 * robot class
 *
 * Provides an interface to control the Fetch
 *
 */

class robot {
public:
    robot(ros::NodeHandle& nh);
    std::map<std::string, transform3> get_link_transforms();
    std::vector<std::string> get_link_names();

    static const std::string ROBOT_NAME;

private:
    std::set<std::string> LINKS_OF_INTEREST;

    ros::NodeHandle& n;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener;

    robot_model model;
    ompl::geometric::SimpleSetup ompl_ss;
};

#endif
#endif
