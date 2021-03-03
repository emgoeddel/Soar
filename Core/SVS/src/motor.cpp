#ifdef ENABLE_ROS

#include "motor.h"

#include <ompl/geometric/planners/rrt/RRTConnect.h>

planning_problem::planning_problem(int qid,
                                   motor_query q,
                                   motor_state* msp,
                                   robot_model* m) : query_id(qid),
                                                     query(q),
                                                     ms(msp),
                                                     model(m)
{
    joint_group = query.soar_query.joint_group;
    if (joint_group == "") joint_group = m->default_joint_group;

    // construct vector state space based on default joint group
    int dof = m->joint_groups[joint_group].size();
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(dof));

    // set the bounds for state space based on joint limits
    ompl::base::RealVectorBounds bounds(dof);
    std::vector<std::string>::iterator i = m->joint_groups[joint_group].begin();
    int b = 0;
    for (; i != m->joint_groups[joint_group].end(); i++)
    {
        std::string j = *i;
        if (m->all_joints[j].type != CONTINUOUS) {
            bounds.setLow(b, m->all_joints[j].min_pos);
            bounds.setHigh(b, m->all_joints[j].max_pos);
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
    std::vector<std::string>::iterator j = m->joint_groups[joint_group].begin();
    int c = 0;
    for (; j != m->joint_groups[joint_group].end(); j++)
    {
        start[c] = q.start_state[*j];
        c++;
    }
    ompl_ss->setStartState(start);

    std::cout << "Set up to plan for joint group " << joint_group.c_str()
              << ", DOF = " << dof << std::endl;
}

void planning_problem::find_one() {
    std::cout << "Using RRT-Connect to find ONE trajectory." << std::endl;
    ompl::geometric::RRTConnect* rrtc =
        new ompl::geometric::RRTConnect(ompl_ss->getSpaceInformation());
    ompl_ss->setPlanner(ompl::base::PlannerPtr(rrtc));

    vec3 cur_pos = model->end_effector_pos(query.start_state);
    std::cout << "Current ee is " << cur_pos[0] << ", " << cur_pos[1] << ", "
              << cur_pos[2] << std::endl;
    std::vector<double> goal_vec = model->solve_ik(query.soar_query.target_center);

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

    std::vector<std::string>::iterator j = model->joint_groups[joint_group].begin();
    for (; j != model->joint_groups[joint_group].end(); j++) {
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

motor::motor(std::string urdf) {
    model.init(urdf);
    std::cout << model.robot_info();
}

robot_model* motor::get_model_ptr() {
    if (model.name == "none") {
        std::cout << "Error: Robot model not initialized" << std::endl;
        return NULL;
    }
    return &model;
}

std::vector<std::string> motor::get_link_names() {
    std::vector<std::string> link_names;

    for (std::set<std::string>::iterator i = model.links_of_interest.begin();
         i != model.links_of_interest.end(); i++) {
        link_names.push_back(*i);
    }

    return link_names;
}

bool motor::new_planner_query(int id, motor_query q, motor_state* msp) {
    ongoing.push_back(planning_problem(id, q, msp, &model));
    ongoing.back().find_one();
    return true;
}

#endif
