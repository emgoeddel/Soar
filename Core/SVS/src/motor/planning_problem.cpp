#ifdef ENABLE_ROS

#include "planning_problem.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>

planning_problem::planning_problem(int qid,
                                   motor_query q,
                                   motor_state* msp,
                                   std::shared_ptr<robot_model> m) : query_id(qid),
                                                                     query(q),
                                                                     model(m),                                                                                    ms(msp),
                                                                     notified_min_traj(false)
{
    joint_group = query.soar_query.joint_group;
    if (joint_group == "") joint_group = m->get_default_joint_group();
    joints = m->get_joint_group(joint_group);
    MAX_THREADS = std::thread::hardware_concurrency();
    if (MAX_THREADS == 0) {
        std::cout << "Hardware concurrency not computable, defaulting to 4 threads."
                  << std::endl;
        MAX_THREADS = 4;
    }
}

planning_problem::~planning_problem() {
    for (std::vector<std::thread>::iterator i = thread_vec.begin();
         i != thread_vec.end(); i++) {
        i->join();
    }

    for (std::vector<ompl::geometric::SimpleSetup*>::iterator j = ss_vec.begin();
         j != ss_vec.end(); j++) {
        delete *j; // deletes collision checkers along with
    }
}

void planning_problem::start_solve() {
    std::cout << "Starting RRT-Connect with " << MAX_THREADS << " threads." << std::endl;
    ms->query_status_callback(query_id, "running");
    for (int i = 0; i < MAX_THREADS; i++) {
        thread_vec.push_back(std::thread(&planning_problem::run_planner, this));
    }
}

// Note that this implements, via callbacks, the status updates defined in
// find_trajectories.cpp--it notifies the motor_state when a query should be
// considered "running" "continuing" or "complete"
void planning_problem::run_planner() {
    // construct vector state space based on default joint group
    int dof = joints.size();
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(dof));

    // set the bounds for state space based on joint limits
    ompl::base::RealVectorBounds bounds(dof);
    int b = 0;
    for (std::vector<std::string>::iterator i = joints.begin(); i != joints.end(); i++)
    {
        std::string j = *i;
        if (model->get_joint_type(j) != CONTINUOUS) {
            bounds.setLow(b, model->get_joint_min(j));
            bounds.setHigh(b, model->get_joint_max(j));
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

    // add a SimpleSetup object for this planning thread
    ompl::geometric::SimpleSetup* cur_ss;
    ompl::base::PlannerTerminationCondition* cur_ptc;
    {
        std::lock_guard<std::mutex> guard(ss_vec_mtx);
        ss_vec.push_back(new ompl::geometric::SimpleSetup(space));
        cur_ss = ss_vec.back();
        ptc_list.push_back(ompl::base::timedPlannerTerminationCondition(60.0));
        cur_ptc = &(ptc_list.back());
    }

    // create a collision checker for this thread
    cur_ss->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
                                        new collision_checker(cur_ss->getSpaceInformation(),
                                                              model, joint_group)));

    // copy the start state into SimpleSetup
    ompl::base::ScopedState<> start(space);
    int c = 0;
    for (std::vector<std::string>::iterator j = joints.begin(); j != joints.end(); j++)
    {
        start[c] = query.start_state[*j];
        c++;
    }
    cur_ss->setStartState(start);

    // set up the planner
    ompl::geometric::RRTConnect* rrtc =
        new ompl::geometric::RRTConnect(cur_ss->getSpaceInformation());
    cur_ss->setPlanner(ompl::base::PlannerPtr(rrtc));

    // check for whether to continue searching after first search finishes
    bool restart_search = false;

    do {
        // use IK to find a goal state
        //vec3 cur_pos = model->end_effector_pos(query.start_state);
        //std::cout << "Current ee is " << cur_pos[0] << ", " << cur_pos[1] << ", "
        //<< cur_pos[2] << std::endl;
        std::vector<double> goal_vec = model->solve_ik(query.soar_query.target_center);
        if (goal_vec.empty()) { // Didn't find an IK solution
            restart_search = true;
            continue;
        }

        // copy the goal state into SimpleSetup
        ompl::base::ScopedState<> goal(cur_ss->getStateSpace());
        for (int i = 0; i < goal_vec.size(); i++) {
            goal[i] = goal_vec[i];
        }
        cur_ss->setGoalState(goal);

        // run the planner
        ompl::base::PlannerStatus status = cur_ss->solve(*cur_ptc);
        std::cout << "Resulting planner status is " << status.asString() << std::endl;

        bool has_trajectory = false;
        trajectory output_traj;
        int num_solns = 0;

        if (!cur_ss->haveExactSolutionPath()) {
            std::cout << "No path found, no trajectory to add!" << std::endl;
        } else {
            ompl::geometric::PathGeometric pg = cur_ss->getSolutionPath();
            pg.interpolate();
            std::cout << "Interpolated trajectory length is " << pg.getStateCount()
                      << std::endl;

            output_traj = path_to_trajectory(pg, cur_ss);
            has_trajectory = true;

            // notify SVS of new trajectory
            ms->new_trajectory_callback(query_id, output_traj);
        }

        {
            std::lock_guard<std::mutex> guard(soln_mtx);
            if (has_trajectory) solutions.push_back(output_traj);
            num_solns = solutions.size();
        }

        // Once the min_number is reached, change the status but don't stop the search
        if (query.has_min_num() &&
            num_solns >= query.soar_query.min_num &&
            !notified_min_traj) {
            ms->query_status_callback(query_id, "continuing");
            notified_min_traj = true;
        }

        if (!query.has_max_num() || num_solns < query.soar_query.max_num) {
            restart_search = true;
            cur_ss->clear();
        } else { // Once the max_number is reached, kill the search
            ms->query_status_callback(query_id, "complete");
            restart_search = false;
            if (has_trajectory) { // If this is the thread that found the last trajectory,
                                  // it should kill all the other threads
                std::list<ompl::base::PlannerTerminationCondition>::iterator p = ptc_list.begin();
                for (; p != ptc_list.end(); p++) {
                    if (p->eval()) { // Already terminated this thread
                        continue;
                    }
                    p->terminate();
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(5)); // dbg status updates
    } while (restart_search);
}

trajectory planning_problem::path_to_trajectory(ompl::geometric::PathGeometric& geom,
                                                ompl::geometric::SimpleSetup* ompl_ss) {
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
