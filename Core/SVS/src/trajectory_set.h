#ifndef TRAJECTORY_SET_H
#define TRAJECTORY_SET_H

#ifdef ENABLE_ROS

#include <sstream>

#include "moveit_msgs/RobotTrajectory.h"

#include "robot.h"

enum TargetType{
    POINT_TARGET,
    BOX_TARGET,
    SPHERE_TARGET
};

/*
 * query struct
 *
 * Holds information about a planning query
 */

struct query {
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

void to_ros_msg(trajectory& from, moveit_msgs::RobotTrajectory& to);
void from_ros_msg(moveit_msgs::RobotTrajectory& from, trajectory& to);

/*
 * trajectory_set class
 *
 * Holds the currently available/valid trajectories that correspond to
 * the current scene graph in an svs_state.
 *
 */

class trajectory_set {
public:
    trajectory_set();
    bool new_command(int id, query query_info);

private:
    std::vector<double> start_joints;
    std::map<int, query> query_map;
    std::map<int, std::set<trajectory> > traj_map;
};

#endif
#endif
