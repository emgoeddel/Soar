#ifdef ENABLE_ROS

#include "planning_problem.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>

planning_problem::planning_problem(int qid,
                                   motor_query q,
                                   motor_state* msp,
                                   std::shared_ptr<robot_model> m) : query_id(qid),
                                                                     query(q),
                                                                     model(m),                                                                                    ms(msp)
{
    joint_group = query.soar_query.joint_group;
    if (joint_group == "") joint_group = m->get_default_joint_group();
    joints = m->get_joint_group(joint_group);

    // construct vector state space based on default joint group
    int dof = joints.size();
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(dof));

    // set the bounds for state space based on joint limits
    ompl::base::RealVectorBounds bounds(dof);
    int b = 0;
    for (std::vector<std::string>::iterator i = joints.begin(); i != joints.end(); i++)
    {
        std::string j = *i;
        if (m->get_joint_type(j) != CONTINUOUS) {
            bounds.setLow(b, m->get_joint_min(j));
            bounds.setHigh(b, m->get_joint_max(j));
        } else {
            // XXX Continuous joints don't actually have bounds, what to do?
            bounds.setLow(b, -10*M_PI);
            bounds.setHigh(b, 10*M_PI);
        }
        b++;
    }

    bounds.check();
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    // XXX Parameter
    space->setLongestValidSegmentFraction(0.005);
    ompl_ss = new ompl::geometric::SimpleSetup(space);
    cc = new collision_checker(ompl_ss->getSpaceInformation(), m, joint_group);
    ompl_ss->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(cc));

    ompl::base::ScopedState<> start(space);
    int c = 0;
    for (std::vector<std::string>::iterator j = joints.begin(); j != joints.end(); j++)
    {
        start[c] = q.start_state[*j];
        c++;
    }
    ompl_ss->setStartState(start);

    std::cout << "Set up to plan for joint group " << joint_group.c_str()
              << ", DOF = " << dof << std::endl;
}

planning_problem::~planning_problem() {
    planner_thread.join();
    if (ompl_ss) delete ompl_ss; // Deletes the collision checker?
}

void planning_problem::find_one() {
    std::cout << "Using RRT-Connect to find ONE trajectory." << std::endl;
    planner_thread = std::thread(&planning_problem::run_planner, this);
}

void planning_problem::run_planner() {
    ompl::geometric::RRTConnect* rrtc =
        new ompl::geometric::RRTConnect(ompl_ss->getSpaceInformation());
    ompl_ss->setPlanner(ompl::base::PlannerPtr(rrtc));

    vec3 cur_pos = model->end_effector_pos(query.start_state);
    std::cout << "Current ee is " << cur_pos[0] << ", " << cur_pos[1] << ", "
              << cur_pos[2] << std::endl;
    std::vector<double> goal_vec = model->solve_ik(query.soar_query.target_center);
    if (goal_vec.empty()) { // Didn't find an IK solution
        return;
    }

    ompl::base::ScopedState<> goal(ompl_ss->getStateSpace());
    for (int i = 0; i < goal_vec.size(); i++) {
        goal[i] = goal_vec[i];
    }
    ompl_ss->setGoalState(goal);
    ompl::base::PlannerStatus status = ompl_ss->solve(5.0);
    std::cout << "Resulting planner status is " << status.asString() << std::endl;

    if (!ompl_ss->haveExactSolutionPath()) {
        std::cout << "No path found, no trajectory to add!" << std::endl;
    } else {
        ompl::geometric::PathGeometric pg = ompl_ss->getSolutionPath();
        pg.interpolate();
        std::cout << "Interpolated trajectory length is " << pg.getStateCount() << std::endl;

        trajectory output_traj = path_to_trajectory(pg);

        ms->new_trajectory_callback(query_id, output_traj);
    }
}

trajectory planning_problem::path_to_trajectory(ompl::geometric::PathGeometric& geom) {
    std::vector<ompl::base::State*> sv = geom.getStates();
    trajectory t;

    for (std::vector<std::string>::iterator j = joints.begin(); j != joints.end(); j++) {
        t.joints.push_back(*j);
    }

    // XXX Need actual time parameterization
    double fake_time = 0.0;
    std::vector<ompl::base::State*>::iterator i = sv.begin();
    for (; i != sv.end(); i++) {
        ompl::base::ScopedState<> ss(ompl_ss->getStateSpace(), *i);
        t.waypoints.push_back(std::vector<double>());
        for (int i = 0; i < ompl_ss->getStateSpace()->getDimension(); i++) {
            t.waypoints.back().push_back(ss[i]);
        }
        t.times.push_back(fake_time);
        fake_time += 1.0;
    }

    t.length = sv.size();
    return t;
}

#endif
