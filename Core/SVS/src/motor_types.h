#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#ifdef ENABLE_ROS

#include <sstream>
#include "trajectory_msgs/JointTrajectory.h"

#include "mat.h"

enum TargetType {
    POINT_TARGET,
    BOX_TARGET,
    SPHERE_TARGET
};

/*
 * query struct
 *
 * Holds soar-level information about a planning query
 */

struct query {
    std::string joint_group;

    int min_num;
    int max_num;
    double min_time;
    double max_time;

    vec3 target_center;
    TargetType target_type;
    vec3 target_box_size; // used for BOX_TARGET
    double target_sphere_radius; // used for SPHERE_TARGET

    bool use_orientation;
    vec3 orientation;
    bool use_orientation_flex;
    vec3 orientation_flex;

    std::string to_str() {
        std::stringstream ss;

        ss << "Min num: " << min_num << " Max num: " << max_num << std::endl
           << "Min time: " << min_time << " Max time: " << max_time << std::endl
           << "Target: " << target_center[0] << ", " << target_center[1] << ", "
           << target_center[2] << std::endl << "Target type: " << target_type;

        return ss.str();
    }
};

/*
 * motor_query struct
 *
 * Holds motor-level information about a planning query
 */

struct motor_query {
    query soar_query;
    //std::vector<obstacle> obstacles; XXX
    std::map<std::string, double> start_state;
};

/*
 * trajectory struct
 *
 * Holds basic information about a single trajectory
 */

struct trajectory {
    int length; // number of waypoints
    std::string frame; // base transform name
    std::vector<std::string> joints; // in-order names of joints
    std::vector<std::vector<double> > waypoints; // position at each waypoint
    std::vector<double> times; // time from start of each waypoint
};

void to_ros_msg(trajectory& from, trajectory_msgs::JointTrajectory& to);
void from_ros_msg(trajectory_msgs::JointTrajectory& from, trajectory& to);

#endif
#endif
