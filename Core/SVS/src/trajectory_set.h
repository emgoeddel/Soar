#ifndef TRAJECTORY_SET_H
#define TRAJECTORY_SET_H

#ifdef ENABLE_ROS

#include "moveit_msgs/RobotTrajectory.h"

#include "robot.h"

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
    //void add_trajectory(trajectory t);
    //void add_trajectories(std::vector<trajectory> tl);
    //void del_trajectory(int index);
    //void del_all();

private:
    std::vector<double> start_joints;
    // Maybe a vector of targets?
};

#endif
#endif
