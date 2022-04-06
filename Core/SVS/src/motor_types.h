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
    int target_samples;

    vec3 target_center;
    TargetType target_type;
    vec3 target_box_size; // used for BOX_TARGET
    double target_sphere_radius; // used for SPHERE_TARGET

    bool use_orientation;
    vec3 orientation;
    bool use_orientation_flex;
    double orientation_flex;

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
 * obstacle struct
 *
 * Holds geometry-only information about an object in the scene
 */

enum ObstacleType {
    BALL_OBSTACLE,
    BOX_OBSTACLE,
    CONVEX_OBSTACLE,
    NON_OBSTACLE
};


struct obstacle {
    std::string name;

    ObstacleType geometry;
    vec3 box_dim; // used for BOX_OBSTACLE
    double ball_radius; // used for BALL_OBSTACLE
    std::vector<vec3> convex_pts; //used for CONVEX_OBSTACLE

    vec3 translation;
    vec3 rotation;
    vec3 scale;
};

class sgnode;
void from_sgnode(sgnode* node, obstacle& to);

/*
 * motor_query struct
 *
 * Holds motor-level information about a planning query
 */

struct motor_query {
    query soar_query;
    std::vector<obstacle> obstacles;
    transform3 base_pose;
    std::map<std::string, double> start_state;

    // Utility functions
    bool has_min_num() { return soar_query.min_num >= 0; }
    bool has_max_num() { return soar_query.max_num > 1; }
    bool has_min_time() { return soar_query.min_time >= 0; }
    bool has_max_time() { return soar_query.max_time > 0; }
    bool has_target_samples() { return soar_query.target_samples > 0; }
};

enum FailureType {
    START_INVALID,
    GOAL_INVALID,
    PLANNING_FAILURE,
    OTHER_ERROR
};

static const int NUM_FAILURE_TYPES = 4;
std::string ft_to_str(FailureType f);
FailureType int_to_ft(int i);

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
    std::vector<std::vector<double> > velocities; // velocity of each joint at waypoints
    std::vector<std::vector<double> > accelerations; // accel of each joint at waypoints
    std::vector<double> times; // time from start of each waypoint
};

void to_ros_msg(trajectory& from, trajectory_msgs::JointTrajectory& to);
void from_ros_msg(trajectory_msgs::JointTrajectory& from, trajectory& to);

#endif
#endif
