#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#ifdef ENABLE_ROS

#include <map>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>

#include "mat.h"

/*
 * robot class
 *
 * Provides an interface to control the Fetch
 *
 */

class robot {
public:
    robot();
    std::map<std::string, transform3> get_link_transforms();
    std::vector<std::string> get_link_names();

    static const std::string ROBOT_NAME;

private:
    std::set<std::string> LINKS_OF_INTEREST;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener;
};

#endif
#endif
