#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#ifdef ENABLE_ROS

#include <map>
#include <ros/ros.h>
#include <urdf/model.h>
#include <urdf_model/model.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "mat.h"

enum joint_type {
    REVOLUTE,
    CONTINUOUS,
    PRISMATIC,
    FIXED,
    UNSUPPORTED
};

struct joint_info {
    std::string name;

    std::string parent_link;
    std::string child_link;

    joint_type type;
    tf2::Transform origin;

    double max_pos;
    double min_pos;
    double max_velocity;
    double max_effort;
};

struct link_info {
    std::string name;

    std::string parent_joint;
    std::set<std::string> child_joints;

    tf2::Transform collision_origin;
    std::string mesh_file;
};

/*
 * robot_model struct
 *
 * Imports and stores necessary information about links and joints
 * from URDF
 *
 */

struct robot_model {
    robot_model() : name("none") {};

    bool init(std::string robot_desc);
    std::string robot_info();

    std::string name;
    std::string root_link;
    std::string default_joint_group;

    // link information
    std::map<std::string, link_info> all_links;
    std::set<std::string> links_of_interest;

    // joint information
    std::map<std::string, joint_info> all_joints;
    std::map<std::string, std::vector<std::string> > joint_groups;
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

    std::string name() { return model.name;}

private:
    ros::NodeHandle& n;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener;

    robot_model model;
    ompl::geometric::SimpleSetup* ompl_ss;
};

#endif
#endif
