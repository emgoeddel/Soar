#ifndef MOTOR_H
#define MOTOR_H

#ifdef ENABLE_ROS

#include <map>
#include <memory>
#include <mutex>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>

#include "mat.h"
#include "trajectory.h"
#include "robot_model.h"

/*
 * motor class
 *
 * Provides an interface to motion planning and robot kinematics
 *
 */

class planning_problem {
public:
    planning_problem(int qid, motor_query q, robot_model* m);
private:
    int query_id;
    motor_query query;
    std::string joint_group;

    std::shared_ptr<robot_model> model;

    ompl::geometric::SimpleSetup* ompl_ss;

    fcl::BroadPhaseCollisionManager* robot;
    fcl::BroadPhaseCollisionManager* world;
};

/*
 * motor class
 *
 * Provides an interface to motion planning and robot kinematics
 *
 */

class motor {
public:
    motor(std::string urdf);

    robot_model* get_model_ptr() { return &model; }
    std::vector<std::string> get_link_names();

    std::string robot_name() { return model.name; }

    bool new_planner_query(int id, motor_query q);

private:
    robot_model model;

    std::vector<planning_problem> ongoing;
};

#endif
#endif
