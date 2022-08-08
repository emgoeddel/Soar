#ifndef PLANNING_PROBLEM_H
#define PLANNING_PROBLEM_H

#ifdef ENABLE_ROS

#include <vector>
#include <list>
#include <thread>
#include <mutex>
#include <chrono>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalLazySamples.h>

#include "collision.h"
#include "robot_model.h"
#include "motor_state.h"
#include "mat.h"

class svs_goal : public ompl::base::GoalLazySamples {
public:
    svs_goal(ompl::base::SpaceInformationPtr si,
             motor_query mq,
             std::shared_ptr<robot_model> m);

    bool isSatisfied(const ompl::base::State* st) const override;

    friend bool
    sample_svs_goal(const ompl::base::GoalLazySamples* gls, ompl::base::State* st);

private:
    int num_samples;

    TargetType target_type;
    vec3 center; // in terms of the robot's base frame
    vec3 box_size; // used for BOX_TARGET
    double sphere_radius; // used for SPHERE_TARGET

    double torso_jnt_val;

    bool match_orientation;
    vec3 orientation;
    bool orientation_flexible;
    double orientation_tolerance;

    std::shared_ptr<robot_model> model;
    std::vector<std::string> joint_names;
};

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

    void start_solve();
    void stop_solve();
    int get_id() const { return query_id; }

private:
    void run_planner();
    FailureType ompl_status_to_failure_type(ompl::base::PlannerStatus ps);

    trajectory path_to_trajectory(ompl::geometric::PathGeometric& geom,
                                  ompl::geometric::SimpleSetup* ompl_ss);
    // Time parameterization helpers; re-implementation of MoveIt functionality
    void unwind_trajectory(trajectory& t);
    void apply_vel_constraints(trajectory& t,
                               std::vector<double>& time_diff,
                               double max_vel_factor,
                               int slowdown_length,
                               double slowdown_factor);
    void update_trajectory(trajectory& t, std::vector<double>& time_diff);

    int query_id;
    motor_query query;

    std::shared_ptr<robot_model> model;
    motor_state* ms;
    std::string joint_group;
    std::vector<std::string> joints;
    std::map<std::string, double> fixed_joints;

    std::vector<trajectory> solutions;
    std::mutex soln_mtx;
    bool reached_min_tc, reached_min_time;
    bool reached_max_tc, reached_max_time;
    bool agent_stopped;
    bool notified_cont, notified_comp;

    std::chrono::time_point<std::chrono::system_clock> start_time;
    unsigned int MAX_THREADS;
    std::vector<std::thread> thread_vec;

    std::mutex ss_vec_mtx;
    std::vector<ompl::geometric::SimpleSetup*> ss_vec;

    std::mutex ptc_mtx;
    std::list<ompl::base::PlannerTerminationCondition> top_ptcs;
    std::list<ompl::base::PlannerTerminationCondition> agent_ptcs;
    std::list<std::pair<ompl::base::PlannerTerminationCondition,
                        ompl::base::PlannerTerminationCondition> > traj_ct_ptcs;
    std::list<std::pair<ompl::base::PlannerTerminationCondition,
                        ompl::base::PlannerTerminationCondition> > time_ptcs;
};

#endif
#endif
