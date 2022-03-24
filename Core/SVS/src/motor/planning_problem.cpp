#ifdef ENABLE_ROS

#include "planning_problem.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>

double sample_double(double min, double max) {
    std::random_device rd;
    std::default_random_engine dre(rd());
    std::uniform_real_distribution<double> dist(0, 1);

    double d = dist(dre);
    return min + (d * (max - min));
}

bool sample_svs_goal(const ompl::base::GoalLazySamples* gls, ompl::base::State* st) {
    const svs_goal* goal = static_cast<const svs_goal*>(gls);

    if (gls->getStateCount() >= goal->num_samples) return false;

    std::vector<double> jnt_values;
    if (goal->target_type == POINT_TARGET) {
        if (goal->match_orientation) {
            if (!goal->orientation_flexible) {
                jnt_values = goal->model->solve_ik(goal->center, goal->orientation);
            } else {
                double a = sample_double(0, goal->orientation_tolerance);
                vec3 ax = random_axis();

                transform3 rot_xform = transform3('r', vec3(goal->orientation[0],
                                                            goal->orientation[1],
                                                            goal->orientation[2]));
                rot_xform = rot_xform*transform3(ax, a);
                vec3 rpy_sample;
                rot_xform.rotation(rpy_sample);
                jnt_values = goal->model->solve_ik(goal->center, rpy_sample);
            }
        } else {
            jnt_values = goal->model->solve_ik(goal->center);
        }
    } else if (goal->target_type == BOX_TARGET) {
        vec3 xyz_sample;
        for (int i = 0; i < 3; i++) {
            double axis_min = goal->center[i] - (goal->box_size[i] / 2);
            double axis_max = goal->center[i] + (goal->box_size[i] / 2);
            xyz_sample[i] = sample_double(axis_min, axis_max);
        }

        if (goal->match_orientation) {
            if (!goal->orientation_flexible) {
                jnt_values = goal->model->solve_ik(xyz_sample, goal->orientation);
            } else {
                double a = sample_double(0, goal->orientation_tolerance);
                vec3 ax = random_axis();

                transform3 rot_xform = transform3('r', vec3(goal->orientation[0],
                                                            goal->orientation[1],
                                                            goal->orientation[2]));
                rot_xform = rot_xform*transform3(ax, a);
                vec3 rpy_sample;
                rot_xform.rotation(rpy_sample);
                jnt_values = goal->model->solve_ik(goal->center, rpy_sample);
            }
        } else {
            jnt_values = goal->model->solve_ik(xyz_sample);
        }
    } else { // SPHERE_TARGET
        double d = sample_double(0, goal->sphere_radius);
        vec3 ax = random_axis();

        vec3 xyz_sample;
        for (int i = 0; i < 3; i++) {
            xyz_sample[i] = goal->center[i] + (d * ax[i]);
        }

        if (goal->match_orientation) {
            if (!goal->orientation_flexible) {
                jnt_values = goal->model->solve_ik(xyz_sample, goal->orientation);
            } else {
                double a = sample_double(0, goal->orientation_tolerance);
                vec3 ax = random_axis();

                transform3 rot_xform = transform3('r', vec3(goal->orientation[0],
                                                            goal->orientation[1],
                                                            goal->orientation[2]));
                rot_xform = rot_xform*transform3(ax, a);
                vec3 rpy_sample;
                rot_xform.rotation(rpy_sample);
                jnt_values = goal->model->solve_ik(goal->center, rpy_sample);
            }
        } else {
            jnt_values = goal->model->solve_ik(xyz_sample);
        }
    }

    if (jnt_values.size() == 0) {
        return false;
    }

    for (int i = 0; i < jnt_values.size(); i++) {
        st->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = jnt_values[i];
    }

    return true;
}

svs_goal::svs_goal(ompl::base::SpaceInformationPtr si,
                   motor_query mq,
                   std::shared_ptr<robot_model> m)
    : ompl::base::GoalLazySamples(si, sample_svs_goal),
    target_type(mq.soar_query.target_type),
    center(mq.soar_query.target_center),
    box_size(mq.soar_query.target_box_size),
    sphere_radius(mq.soar_query.target_sphere_radius),
    match_orientation(mq.soar_query.use_orientation),
    orientation(mq.soar_query.orientation),
    orientation_flexible(mq.soar_query.use_orientation_flex),
    orientation_tolerance(mq.soar_query.orientation_flex),
    model(m)
{
    if (mq.has_target_samples()) num_samples = mq.soar_query.target_samples;
    else num_samples = 1;

    joint_names = model->get_joint_group(mq.soar_query.joint_group);
}

bool svs_goal::isSatisfied(const ompl::base::State* st) const {
    std::map<std::string, double> joint_vals;
    for (int i = 0; i < si_->getStateDimension(); i++) {
        joint_vals[joint_names[i]] = st->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
    }

    transform3 ee_xform = model->end_effector_xform(joint_vals);

    if (match_orientation) {
        transform3 target_xform('r', orientation);
        if (target_xform.angle_difference(ee_xform) > orientation_tolerance) {
            return false;
        }
    }

    vec3 ee_pos;
    ee_xform.position(ee_pos);

    if (target_type == POINT_TARGET) {
        if (sqrt(pow(ee_pos[0] - center[0], 2) +
                 pow(ee_pos[1] - center[1], 2) +
                 pow(ee_pos[2] - center[2], 2)) > 1e-3) return false; // XXX Tolerance?
    } else if (target_type == BOX_TARGET) {
        for (int i = 0; i < 3; i++) {
            if (ee_pos[i] > center[i] + (box_size[i]/2) ||
                ee_pos[i] < center[i] - (box_size[i]/2)) return false;
        }
    } else { // SPHERE_TARGET
        if (sqrt(pow(ee_pos[0] - center[0], 2) +
                 pow(ee_pos[1] - center[1], 2) +
                 pow(ee_pos[2] - center[2], 2)) > sphere_radius) return false;

    }

    return true;
}

planning_problem::planning_problem(int qid,
                                   motor_query q,
                                   motor_state* msp,
                                   std::shared_ptr<robot_model> m) : query_id(qid),
                                                                     query(q),
                                                                     model(m),                                                                                    ms(msp),
                                                                     reached_min_tc(false),
                                                                     reached_min_time(false),
                                                                     reached_max_tc(false),
                                                                     reached_max_time(false),
                                                                     agent_stopped(false),
                                                                     notified_cont(false),
                                                                     notified_comp(false)
{
    joint_group = query.soar_query.joint_group;
    if (joint_group == "") joint_group = m->get_default_joint_group();
    joints = m->get_joint_group(joint_group);
    MAX_THREADS = std::thread::hardware_concurrency();
    //MAX_THREADS = 1; dbg
    if (MAX_THREADS == 0) {
        std::cout << "Hardware concurrency not computable, defaulting to 4 threads."
                  << std::endl;
        MAX_THREADS = 4;
    }
}

planning_problem::~planning_problem() {
    // Kill all the searches
    {
        std::lock_guard<std::mutex> guard1(ptc_mtx);
        std::list<ompl::base::PlannerTerminationCondition>::iterator p = top_ptcs.begin();
        for (; p != top_ptcs.end(); p++) {
            if (!p->eval()) p->terminate();
        }
    }

    // Wait for the threads to join
    for (std::vector<std::thread>::iterator i = thread_vec.begin();
         i != thread_vec.end(); i++) {
        if (i->joinable()) i->join();
    }

    // Delete all the setups
    for (std::vector<ompl::geometric::SimpleSetup*>::iterator j = ss_vec.begin();
         j != ss_vec.end(); j++) {
        delete *j; // deletes collision checkers along with
    }
}

void planning_problem::start_solve() {
    std::cout << "Starting RRT-Connect with " << MAX_THREADS << " threads." << std::endl;
    ms->query_status_callback(query_id, "running");

    start_time = std::chrono::system_clock::now();
    for (int i = 0; i < MAX_THREADS; i++) {
        thread_vec.push_back(std::thread(&planning_problem::run_planner, this));
    }
}

void planning_problem::stop_solve() {
    {
        std::lock_guard<std::mutex> guard1(ptc_mtx);
        std::list<ompl::base::PlannerTerminationCondition>::iterator p = agent_ptcs.begin();
        for (; p != agent_ptcs.end(); p++) {
            if (!p->eval()) p->terminate();
        }
        agent_stopped = true;
    }

    for (std::vector<std::thread>::iterator i = thread_vec.begin();
         i != thread_vec.end(); i++) {
        if (i->joinable()) i->join();
    }
    thread_vec.clear();
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
        std::lock_guard<std::mutex> guard1(ss_vec_mtx);
        ss_vec.push_back(new ompl::geometric::SimpleSetup(space));
        cur_ss = ss_vec.back();
    }

    {
        std::lock_guard<std::mutex> guard2(ptc_mtx);
        // pair for min, max trajectory limits
        traj_ct_ptcs.push_back(std::make_pair(ompl::base::plannerNonTerminatingCondition(),
                                              ompl::base::plannerNonTerminatingCondition()));

        // pair for min, max time limits
        time_ptcs.push_back(std::make_pair(ompl::base::plannerNonTerminatingCondition(),
                                           ompl::base::plannerNonTerminatingCondition()));

        agent_ptcs.push_back(ompl::base::plannerNonTerminatingCondition());

        // agent || (time > min && num_traj > min) && (time > max || num_traj > max)
        top_ptcs.push_back(
            ompl::base::plannerOrTerminationCondition(
                agent_ptcs.back(),
                ompl::base::plannerAndTerminationCondition(
                    ompl::base::plannerAndTerminationCondition(time_ptcs.back().first,
                                                               traj_ct_ptcs.back().first),
                    ompl::base::plannerOrTerminationCondition(time_ptcs.back().second,
                                                              traj_ct_ptcs.back().second))));
        cur_ptc = &(top_ptcs.back());
    }

    // create a collision checker for this thread
    cur_ss->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
                                        new collision_checker(cur_ss->getSpaceInformation(),
                                                              model, query.base_pose,
                                                              joint_group,
                                                              query.obstacles)));

    // set up the planner
    ompl::geometric::RRTConnect* rrtc =
        new ompl::geometric::RRTConnect(cur_ss->getSpaceInformation());
    cur_ss->setPlanner(ompl::base::PlannerPtr(rrtc));

    // copy the start state into SimpleSetup
    ompl::base::ScopedState<> start(space);
    int c = 0;
    for (std::vector<std::string>::iterator j = joints.begin(); j != joints.end(); j++)
    {
        start[c] = query.start_state[*j];
        c++;
    }
    cur_ss->setStartState(start);

    ompl::base::GoalPtr g(new svs_goal(cur_ss->getSpaceInformation(), query, model));
    cur_ss->setGoal(g);

    // check for whether to continue searching after first search finishes
    bool restart_search = false;

    do {
        g->as<ompl::base::GoalLazySamples>()->clear();
        cur_ss->clear();

        // run the planner
        g->as<ompl::base::GoalLazySamples>()->startSampling();
        ompl::base::PlannerStatus status = cur_ss->solve(*cur_ptc);

        g->as<ompl::base::GoalLazySamples>()->stopSampling();

        bool has_trajectory = false;
        trajectory output_traj;
        int num_solns = 0;

        if (!cur_ss->haveExactSolutionPath()) {
            if (!agent_stopped && !notified_comp)
                ms->failure_callback(query_id, ompl_status_to_failure_type(status));
        } else {
            ompl::geometric::PathGeometric pg = cur_ss->getSolutionPath();
            pg.interpolate();

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

        {
            std::lock_guard<std::mutex> guard(ptc_mtx);
            if (top_ptcs.front().eval()) {
                restart_search = false;

                if (agent_ptcs.front().eval())
                    ms->query_status_callback(query_id, "stopped");

                continue;
            }
        }

        std::chrono::duration<double> tpt = std::chrono::system_clock::now() - start_time;

        // Check for min time
        if (!reached_min_time &&
            ((query.has_min_time() && tpt.count() > query.soar_query.min_time) ||
             !query.has_min_time())) {
            reached_min_time = true;
            // Tell all the time PTCs that the minimum has been reached
            std::lock_guard<std::mutex> guard(ptc_mtx);
            std::list<std::pair<ompl::base::PlannerTerminationCondition,
                                ompl::base::PlannerTerminationCondition> >::iterator p =
                time_ptcs.begin();
            for (; p != time_ptcs.end(); p++) {
                if (!p->first.eval())
                    p->first.terminate(); // first is for min
            }
        }

        // Check for min number of trajectories
        if (!reached_min_tc &&
            ((query.has_min_num() && num_solns >= query.soar_query.min_num) ||
             !query.has_min_num())) {
            reached_min_tc = true;
            // Tell all the tc PTCs that the minimum has been reached
            std::lock_guard<std::mutex> guard(ptc_mtx);
            std::list<std::pair<ompl::base::PlannerTerminationCondition,
                                ompl::base::PlannerTerminationCondition> >::iterator p =
                traj_ct_ptcs.begin();
            for (; p != traj_ct_ptcs.end(); p++) {
                if (!p->first.eval())
                    p->first.terminate(); // first is for min
            }
        }

        // Check for max time
        if (!reached_max_time && query.has_max_time() &&
            tpt.count() > query.soar_query.max_time) {
            reached_max_time = true;
            // Tell all the time PTCs that the maximum has been reached
            std::lock_guard<std::mutex> guard(ptc_mtx);
            std::list<std::pair<ompl::base::PlannerTerminationCondition,
                                ompl::base::PlannerTerminationCondition> >::iterator p =
                time_ptcs.begin();
            for (; p != time_ptcs.end(); p++) {
                if (!p->second.eval())
                    p->second.terminate(); // second is for max
            }
        }

        // Check for max number of trajectories
        if (!reached_max_tc && query.has_max_num() &&
            num_solns >= query.soar_query.max_num) {
            reached_max_tc = true;
            // Tell all the tc PTCs that the maximum has been reached
            std::lock_guard<std::mutex> guard(ptc_mtx);
            std::list<std::pair<ompl::base::PlannerTerminationCondition,
                                ompl::base::PlannerTerminationCondition> >::iterator p =
                traj_ct_ptcs.begin();
            for (; p != traj_ct_ptcs.end(); p++) {
                if (!p->second.eval())
                    p->second.terminate(); // second is for max
            }
        }

        bool reached_max = (reached_max_tc || reached_max_time);

        if (reached_min_tc && reached_min_time && !reached_max && !notified_cont) {
            notified_cont = true;
            ms->query_status_callback(query_id, "continuing");
        }

        if (reached_max) {
            if (!notified_comp) {
                notified_comp = true;
                ms->query_status_callback(query_id, "complete");
            }
            restart_search = false;
        } else restart_search = true;

        // {
        //     std::lock_guard<std::mutex> guard(ptc_mtx);

        //     std::list<ompl::base::PlannerTerminationCondition>::iterator top =
        //         top_ptcs.begin();
        //     std::list<std::pair<ompl::base::PlannerTerminationCondition,
        //                         ompl::base::PlannerTerminationCondition> >::iterator ct =
        //         traj_ct_ptcs.begin();
        //     std::list<std::pair<ompl::base::PlannerTerminationCondition,
        //                         ompl::base::PlannerTerminationCondition> >::iterator time =
        //         time_ptcs.begin();

        //     for (; top != top_ptcs.end(); top++) {
        //         std::cout << "TOP: ";
        //         if (top->eval()) std::cout << "KILL ";
        //         else std::cout << "continue ";

        //         std::cout << "TRAJ CTS: ";
        //         if (ct->first.eval()) std::cout << "KILL ";
        //         else std::cout << "continue ";
        //         std::cout << "/ ";
        //         if (ct->second.eval()) std::cout << "KILL ";
        //         else std::cout << "continue ";

        //         std::cout << "TIME CTS: ";
        //         if (time->first.eval()) std::cout << "KILL ";
        //         else std::cout << "continue ";
        //         std::cout << "/ ";
        //         if (time->second.eval()) std::cout << "KILL" << std::endl;
        //         else std::cout << "continue" << std::endl;

        //         ct++;
        //         time++;
        //     }
        // }

        //std::this_thread::sleep_for(std::chrono::seconds(2)); // dbg status updates
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

FailureType planning_problem::ompl_status_to_failure_type(ompl::base::PlannerStatus ps) {
    if (ps == ompl::base::PlannerStatus::INVALID_START) return START_INVALID;

    if (ps == ompl::base::PlannerStatus::INVALID_GOAL ||
        ps == ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE) return GOAL_INVALID;

    if (ps == ompl::base::PlannerStatus::TIMEOUT ||
        ps == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) return PLANNING_FAILURE;

    return OTHER_ERROR;
}

#endif
