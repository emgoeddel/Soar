#ifndef PLANNING_PROBLEM_H
#define PLANNING_PROBLEM_H

#ifdef ENABLE_ROS

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "collision.h"
#include "robot_model.h"
#include "motor_state.h"
#include "mat.h"

/*
 * Planning_problem class
 *
 * Holds the search space and collision info for a particular planning
 * query and runs the search through an OMPL SimpleSetup with FCL collision
 *
 */

class planning_problem {
public:
    planning_problem(int qid, motor_query q,
                     motor_state* msp,
                     std::shared_ptr<robot_model> m);
    ~planning_problem();

    void find_one();

private:
    trajectory path_to_trajectory(ompl::geometric::PathGeometric& geom);

    int query_id;
    motor_query query;

    std::shared_ptr<robot_model> model;
    motor_state* ms;
    std::string joint_group;
    std::vector<std::string> joints;

    ompl::geometric::SimpleSetup* ompl_ss;
    collision_checker* cc;
};

#endif
#endif
