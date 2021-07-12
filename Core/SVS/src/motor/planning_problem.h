#ifndef PLANNING_PROBLEM_H
#define PLANNING_PROBLEM_H

#ifdef ENABLE_ROS

#include <vector>
#include <list>
#include <thread>
#include <mutex>

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

    void start_solve(int num_solutions = 1);

private:
    void run_planner();
    trajectory path_to_trajectory(ompl::geometric::PathGeometric& geom,
                                  ompl::geometric::SimpleSetup* ompl_ss);

    int query_id;
    motor_query query;

    std::shared_ptr<robot_model> model;
    motor_state* ms;
    std::string joint_group;
    std::vector<std::string> joints;

    std::vector<trajectory> solutions;
    std::mutex soln_mtx;

    unsigned int MAX_THREADS;
    std::vector<std::thread> thread_vec;
    std::mutex ss_vec_mtx;
    std::vector<ompl::geometric::SimpleSetup*> ss_vec;
    std::list<ompl::base::PlannerTerminationCondition> ptc_list;
};

#endif
#endif
