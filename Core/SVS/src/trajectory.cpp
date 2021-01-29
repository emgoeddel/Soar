#ifdef ENABLE_ROS

#include "trajectory.h"

void to_ros_msg(trajectory& from, moveit_msgs::RobotTrajectory& to) {
    to.joint_trajectory.header.frame_id = from.frame;
    to.joint_trajectory.joint_names = from.joints;
    to.joint_trajectory.points.clear();
    for (int i = 0; i < from.length; i++) {
        trajectory_msgs::JointTrajectoryPoint p;
        p.time_from_start = ros::Duration(from.times[i]);
        p.positions = from.waypoints[i];
        to.joint_trajectory.points.push_back(p);
    }

    trajectory_msgs::MultiDOFJointTrajectory empty;
    to.multi_dof_joint_trajectory = empty;
}

void from_ros_msg(moveit_msgs::RobotTrajectory& from, trajectory& to) {
    to.length = from.joint_trajectory.points.size();
    to.frame = from.joint_trajectory.header.frame_id;
    to.joints = from.joint_trajectory.joint_names;
    for (int i = 0; i < from.joint_trajectory.points.size(); i++) {
        trajectory_msgs::JointTrajectoryPoint p = from.joint_trajectory.points[i];
        to.waypoints.push_back(p.positions);
        to.times.push_back(p.time_from_start.toSec());
    }

    if (from.multi_dof_joint_trajectory.points.size() > 0) {
        std::cout << "Warning: Discarding MultiDOFJointTrajectory from ROS message."
                  << std::endl;
    }
}

trajectory_set::trajectory_set(motor* m) : mtr(m) {}

void trajectory_set::copy_from(trajectory_set* other) {
    queries.clear();
    // XXX This isn't quite right
    // How to deal with trajectories from parent state?
    trajectories = other->trajectories;
}

void trajectory_set::new_query(int id, query q) {
    queries[id] = q;
    std::cout << "Added a query to a trajectory set! Need to talk to motor!" << std::endl;
}

#endif
