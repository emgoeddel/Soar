#ifdef ENABLE_ROS

#include "motor_types.h"
#include "motor.h"

void trajectory::copy_from(trajectory& other) {
    length = other.length;
    frame = other.frame;
    joints = other.joints;
    waypoints = other.waypoints;
    times = other.times;
}

void to_ros_msg(trajectory& from, trajectory_msgs::JointTrajectory& to) {
    to.header.frame_id = from.frame;
    to.joint_names = from.joints;
    to.points.clear();
    for (int i = 0; i < from.length; i++) {
        trajectory_msgs::JointTrajectoryPoint p;
        p.time_from_start = ros::Duration(from.times[i]);
        p.positions = from.waypoints[i];
        to.points.push_back(p);
    }
}

void from_ros_msg(trajectory_msgs::JointTrajectory& from, trajectory& to) {
    to.length = from.points.size();
    to.frame = from.header.frame_id;
    to.joints = from.joint_names;
    for (int i = 0; i < from.points.size(); i++) {
        trajectory_msgs::JointTrajectoryPoint p = from.points[i];
        to.waypoints.push_back(p.positions);
        to.times.push_back(p.time_from_start.toSec());
    }
}

#endif
