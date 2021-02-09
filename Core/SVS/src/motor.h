#ifndef MOTOR_H
#define MOTOR_H

#ifdef ENABLE_ROS

#include <map>
#include <memory>
#include <mutex>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "mat.h"
#include "motor_state.h"
#include "collision.h"


/*
 * planning_problem class
 *
 * Holds the search space and collision info for a particular planning
 * query and runs the search through an OMPL SimpleSetup with FCL collision
 *
 */

class planning_problem {
public:
    planning_problem(int qid, motor_query q, motor_state* msp, robot_model* m);
    void find_one();

private:
    trajectory path_to_trajectory(ompl::geometric::PathGeometric& geom);

    int query_id;
    motor_query query;

    std::shared_ptr<robot_model> model;
    std::shared_ptr<motor_state> ms;
    std::string joint_group;

    ompl::geometric::SimpleSetup* ompl_ss;
    collision_checker* cc;
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

    robot_model* get_model_ptr();
    std::vector<std::string> get_link_names();

    std::string robot_name() { return model.name; }

    bool new_planner_query(int id, motor_query q, motor_state* msp);

private:
    robot_model model;
    std::vector<planning_problem> ongoing;
};

#endif
#endif
