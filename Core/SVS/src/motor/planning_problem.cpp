#ifdef ENABLE_ROS

#include "planning_problem.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>

// For eval
#include <pthread.h>
#include <time.h>
#include "timespec/timespec.h"
#include <fstream>

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
    if (gls->samplingAttemptsCount() > 20 && gls->getStateCount() == 0) return false;

    std::vector<double> jnt_values;
    if (goal->target_type == POINT_TARGET) {
        if (goal->match_orientation) {
            if (!goal->orientation_flexible) {
                jnt_values = goal->model->solve_ik(goal->center,
                                                   goal->orientation,
                                                   goal->torso_jnt_val);
            } else {
                double a = sample_double(0, goal->orientation_tolerance);
                vec3 ax = random_axis();

                transform3 rot_xform = transform3('r', vec3(goal->orientation[0],
                                                            goal->orientation[1],
                                                            goal->orientation[2]));
                rot_xform = rot_xform*transform3(ax, a);
                vec3 rpy_sample;
                rot_xform.rotation(rpy_sample);
                jnt_values = goal->model->solve_ik(goal->center,
                                                   rpy_sample,
                                                   goal->torso_jnt_val);
            }
        } else {
            jnt_values = goal->model->solve_ik(goal->center, goal->torso_jnt_val);
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
                jnt_values = goal->model->solve_ik(xyz_sample,
                                                   goal->orientation,
                                                   goal->torso_jnt_val);
            } else {
                double a = sample_double(0, goal->orientation_tolerance);
                vec3 ax = random_axis();

                transform3 rot_xform = transform3('r', vec3(goal->orientation[0],
                                                            goal->orientation[1],
                                                            goal->orientation[2]));
                rot_xform = rot_xform*transform3(ax, a);
                vec3 rpy_sample;
                rot_xform.rotation(rpy_sample);
                jnt_values = goal->model->solve_ik(goal->center,
                                                   rpy_sample,
                                                   goal->torso_jnt_val);
            }
        } else {
            jnt_values = goal->model->solve_ik(xyz_sample, goal->torso_jnt_val);
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
                jnt_values = goal->model->solve_ik(xyz_sample,
                                                   goal->orientation,
                                                   goal->torso_jnt_val);
            } else {
                double a = sample_double(0, goal->orientation_tolerance);
                vec3 ax = random_axis();

                transform3 rot_xform = transform3('r', vec3(goal->orientation[0],
                                                            goal->orientation[1],
                                                            goal->orientation[2]));
                rot_xform = rot_xform*transform3(ax, a);
                vec3 rpy_sample;
                rot_xform.rotation(rpy_sample);
                jnt_values = goal->model->solve_ik(goal->center,
                                                   rpy_sample,
                                                   goal->torso_jnt_val);
            }
        } else {
            jnt_values = goal->model->solve_ik(xyz_sample, goal->torso_jnt_val);
        }
    }

    if (jnt_values.size() == 0) {
        return false;
    }

    std::vector<std::string> jnt_names = goal->model->get_joint_group("arm");
    // Unwrap continuous joints to fit into vector space limits
    for (int i = 0; i < jnt_values.size(); i++) {

        if (goal->model->get_joint_type(jnt_names[i]) != CONTINUOUS) {
            continue;
        }

        while (jnt_values[i] > 2*M_PI) {
            jnt_values[i] -= 2*M_PI;
        }
        while (jnt_values[i] < -2*M_PI) {
            jnt_values[i] += 2*M_PI;
        }
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
    torso_jnt_val = mq.start_state["torso_lift_joint"]; // Only fixed joint needed for IK
    if (torso_jnt_val == 0.0) {
        std::cout << "WARNING: Torso joint value set to zero in planning system!"
                  <<std::endl;
    }

    // Convert goal to robot's base frame if necessary
    // All other calculations in this class assume this has been done
    if (mq.soar_query.target_frame == "self") {
        center = mq.soar_query.target_center;
    } else if (mq.soar_query.target_frame == "world") {
        transform3 world_to_self = mq.base_pose.inv();
        center = world_to_self(mq.soar_query.target_center);
    } else {
        std::cout << "ERROR: Unsupported target frame given; defaulting to world frame."
                  << std::endl;
        transform3 world_to_self = mq.base_pose.inv();
        center = world_to_self(mq.soar_query.target_center);
    }
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
                                                                     notified_comp(false),
                                                                     solve_time(-1)
{
    joint_group = query.soar_query.joint_group;
    if (joint_group == "") joint_group = m->get_default_joint_group();
    joints = m->get_joint_group(joint_group);

    std::map<std::string, double>::iterator i = query.start_state.begin();
    for (; i != query.start_state.end(); i++) {
        bool is_fixed = true;
        std::vector<std::string>::iterator j = joints.begin();
        for (; j != joints.end(); j++) {
            if (*j == i->first) {
                is_fixed = false;
                break;
            }
        }
        if (is_fixed) fixed_joints[i->first] = i->second;
    }

    MAX_THREADS = std::thread::hardware_concurrency();
    // MAX_THREADS = 1; //dbg
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

double planning_problem::get_solve_time() {
    std::lock_guard<std::mutex> guard1(st_mtx);
    return solve_time;
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
            bounds.setLow(b, -4*M_PI);
            bounds.setHigh(b, 4*M_PI);
        }
        b++;
    }

    bounds.check();
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    // XXX Parameter
    space->setLongestValidSegmentFraction(0.001);

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
    // cur_ss->setStateValidityChecker
    //     (ompl::base::StateValidityCheckerPtr(
    //         new collision_checker(cur_ss->getSpaceInformation(),
    //                               model, query.base_pose,
    //                               joint_group,
    //                               query.obstacles,
    //                               query.start_state["torso_lift_joint"])));
    collision_checker* cc;
    if (query.soar_query.holding_object) {
        cc = new collision_checker(cur_ss->getSpaceInformation(),
                                   model, query.base_pose,
                                   joint_group, fixed_joints,
                                   query.obstacles, query.held_object);
    } else {
        cc = new collision_checker(cur_ss->getSpaceInformation(),
                                   model, query.base_pose,
                                   joint_group, fixed_joints,
                                   query.obstacles);
    }
    cur_ss->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(cc));

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

        // time eval
        pthread_t t_id = pthread_self();
        clockid_t c_id;
        int err = pthread_getcpuclockid(t_id, &c_id);

        if (err) std::cout << "Unable to get clock id for thread "
                           << t_id << "!" << std::endl;

        timespec start_plan;
        err = clock_gettime(c_id, &start_plan);
        if (err) std::cout << "Unable to get planning start time for thread "
                           << t_id << "!" << std::endl;

        // run the planner
        g->as<ompl::base::GoalLazySamples>()->startSampling();

        ompl::base::PlannerStatus status = cur_ss->solve(*cur_ptc);

        timespec end_plan;
        err = clock_gettime(c_id, &end_plan);
        if (err) std::cout << "Unable to get planning end time for thread "
                           << t_id << "!" << std::endl;

        timespec plan_time = timespec_sub(end_plan, start_plan);

        g->as<ompl::base::GoalLazySamples>()->stopSampling();

        bool has_trajectory = false;
        trajectory output_traj;
        int num_solns = 0;

        if (!cur_ss->haveExactSolutionPath()) {
            if (!agent_stopped && !notified_comp) {
                if (!g->as<ompl::base::GoalLazySamples>()->hasStates())
                    ms->failure_callback(query_id, GOAL_INVALID);
                else ms->failure_callback(query_id, ompl_status_to_failure_type(status));
            }
        } else {
            // time eval
            timespec start_post;
            err = clock_gettime(c_id, &start_post);
            if (err) std::cout << "Unable to get postprocess start time for thread "
                               << t_id << "!" << std::endl;

            cur_ss->simplifySolution();
            ompl::geometric::PathGeometric pg = cur_ss->getSolutionPath();
            pg.interpolate();

            // bool found_problem = false;
            // for (int s = 0; s < pg.getStateCount(); s++) {
            //     if (!cc->isValid(pg.getState(s))) {
            //         found_problem = true;
            //         std::cout << "|-------------------------------------|" << std::endl
            //                   << "|                                     |" << std::endl
            //                   << "|     COLLISION IN TRAJECTORY         |" << std::endl
            //                   << "|                                     |" << std::endl
            //                   << "|-------------------------------------|" << std::endl;
            //     }
            // }
            // if (!found_problem) {
            //     std::cout << "Checked " << pg.getStateCount() << " waypoints for collision"
            //               << std::endl;
            // }

            output_traj = path_to_trajectory(pg, cur_ss);
            output_traj.holding_object = query.soar_query.holding_object;
            if (output_traj.holding_object) output_traj.held_object = query.held_object;
            has_trajectory = true;

            timespec end_post;
            err = clock_gettime(c_id, &end_post);
            if (err) std::cout << "Unable to get postprocess end time for thread "
                               << t_id << "!" << std::endl;

            timespec post_time = timespec_sub(end_plan, start_plan);
            timespec total_time = timespec_add(plan_time, post_time);

            output_traj.planning_time = timespec_to_double(total_time);

            // notify SVS of new trajectory
            ms->new_trajectory_callback(query_id, output_traj);
        }

        {
            std::lock_guard<std::mutex> guard(soln_mtx);
            if (has_trajectory) solutions.push_back(output_traj);
            num_solns = solutions.size();
        }

        std::chrono::duration<double> tpt = std::chrono::system_clock::now() - start_time;

        {
            std::lock_guard<std::mutex> guard(ptc_mtx);
            if (top_ptcs.front().eval()) {
                restart_search = false;

                if (agent_ptcs.front().eval())
                    ms->query_status_callback(query_id, "stopped");

                std::lock_guard<std::mutex> guard2(st_mtx);
                solve_time = tpt.count();
                continue;
            }
        }

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

                std::lock_guard<std::mutex> guard2(st_mtx);
                solve_time = tpt.count();
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

        // std::this_thread::sleep_for(std::chrono::seconds(2)); // dbg status updates
    } while (restart_search);
}

FailureType planning_problem::ompl_status_to_failure_type(ompl::base::PlannerStatus ps) {
    if (ps == ompl::base::PlannerStatus::INVALID_START) return START_INVALID;

    if (ps == ompl::base::PlannerStatus::INVALID_GOAL ||
        ps == ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE) return GOAL_INVALID;

    if (ps == ompl::base::PlannerStatus::TIMEOUT ||
        ps == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) return PLANNING_FAILURE;

    return OTHER_ERROR;
}

trajectory planning_problem::path_to_trajectory(ompl::geometric::PathGeometric& geom,
                                                ompl::geometric::SimpleSetup* ompl_ss) {
    std::vector<ompl::base::State*> sv = geom.getStates();
    trajectory t;

    for (std::vector<std::string>::iterator j = joints.begin(); j != joints.end(); j++) {
        t.joints.push_back(*j);
    }

    if (joint_group == "arm" && query.start_state.count("torso_lift_joint")) {
        t.fixed_joints["torso_lift_joint"] = query.start_state["torso_lift_joint"];
    }
    if (joint_group == "arm" && query.start_state.count("l_gripper_finger_joint")) {
        t.fixed_joints["l_gripper_finger_joint"] =
            query.start_state["l_gripper_finger_joint"];
    }
    if (joint_group == "arm" && query.start_state.count("r_gripper_finger_joint")) {
        t.fixed_joints["r_gripper_finger_joint"] =
            query.start_state["r_gripper_finger_joint"];
    }

    std::vector<ompl::base::State*>::iterator i = sv.begin();
    for (; i != sv.end(); i++) {
        ompl::base::ScopedState<> ss(ompl_ss->getStateSpace(), *i);
        t.waypoints.push_back(std::vector<double>());
        for (int i = 0; i < ompl_ss->getStateSpace()->getDimension(); i++) {
            t.waypoints.back().push_back(ss[i]);
        }
    }
    t.length = t.waypoints.size();

    unwind_trajectory(t);

    std::vector<double> time_diff(t.length-1, 0.0);

    apply_vel_constraints(t, time_diff, 0.2, 20, 8.0);
    update_trajectory(t, time_diff);

    return t;
}

void planning_problem::unwind_trajectory(trajectory& t) {
    if (t.length == 0) return;

    for (int j = 0; j != t.joints.size(); j++) {
        if (model->get_joint_type(t.joints[j]) != CONTINUOUS)
            continue;

        double running_offset = 0.0;
        double last_value = t.waypoints[0][j];

        for (int w = 1; w != t.length; w++) {
            double current_value = t.waypoints[w][j];

            if (last_value > current_value + M_PI) running_offset += (2 * M_PI);
            else if (current_value > last_value + M_PI) running_offset -= (2 * M_PI);

            last_value = current_value;
            if (running_offset > std::numeric_limits<double>::epsilon() ||
                running_offset < -std::numeric_limits<double>::epsilon()) {
                current_value += running_offset;
                t.waypoints[w][j] = current_value;
            }
        }
    }
}

void planning_problem::apply_vel_constraints(trajectory& t,
                                             std::vector<double>& time_diff,
                                             double max_vel_factor,
                                             int slowdown_length,
                                             double slowdown_factor)
{
    // Which diffs are included in the end slowdown
    double slowdown_start = (t.length - 2) - slowdown_length;
    if (slowdown_start < 0) slowdown_start = 0;
    double m = (slowdown_factor - 1.0) / double(slowdown_length);

    for (int i = 0; i < t.length - 1; i++)
    {
        std::vector<double> curr_waypoint = t.waypoints[i];
        std::vector<double> next_waypoint = t.waypoints[i + 1];

        for (int j = 0; j < t.joints.size(); j++)
        {
            double v_max = model->get_joint_max_velocity(t.joints[j]) * max_vel_factor;
            const double dq1 = curr_waypoint[j];
            const double dq2 = next_waypoint[j];
            const double t_min = std::abs(dq2 - dq1) / v_max;
            if (t_min > time_diff[i])
                time_diff[i] = t_min;
        }

        // Implements linear slowdown to last diff
        if (i >= slowdown_start) {
            double x = (double)i - slowdown_start;
            double scaling = m*x + 1.0;
            time_diff[i] = time_diff[i]*scaling;
        }
    }
}

// Assumes trajectory t holds waypoints already; fills in times, velocities, and
// accelerations based on the given time_diff vector
void planning_problem::update_trajectory(trajectory& t, std::vector<double>& time_diff) {
    if (time_diff.empty()) return;

    double time_sum = 0.0;


    int num_points = t.length;
    t.times = std::vector<double>(num_points);
    t.times[0] = time_sum;

    // Times
    for (int i = 1; i < num_points; i++) {
        time_sum += time_diff[i-1];
        t.times[i] = time_sum;
    }

    // Return if there is only one point in the trajectory!
    if (num_points <= 1) return;

    int prev_waypoint = 0;
    int curr_waypoint = 0;
    int next_waypoint = 0;

    t.velocities = std::vector<std::vector<double> >(num_points);
    t.accelerations = std::vector<std::vector<double> >(num_points);

    // Accelerations and velocities
    for (int k = 0; k < num_points; k++) {
        t.velocities[k] = std::vector<double>(t.joints.size());
        t.accelerations[k] = std::vector<double>(t.joints.size());
    }

    for (; curr_waypoint < num_points; curr_waypoint++) {
        if (curr_waypoint > 0) prev_waypoint = curr_waypoint - 1;

        if (curr_waypoint < num_points - 1)
            next_waypoint = curr_waypoint + 1;

        for (int j = 0; j < t.joints.size(); j++) {
            double q1;
            double q2;
            double q3;
            double dt1;
            double dt2;

            if (curr_waypoint == 0) {
                // First point
                q1 = t.waypoints[next_waypoint][j];
                q2 = t.waypoints[curr_waypoint][j];
                q3 = q1;

                dt1 = dt2 = time_diff[curr_waypoint];
            } else if (curr_waypoint < num_points - 1) {
                // Middle points
                q1 = t.waypoints[prev_waypoint][j];
                q2 = t.waypoints[curr_waypoint][j];
                q3 = t.waypoints[next_waypoint][j];

                dt1 = time_diff[curr_waypoint - 1];
                dt2 = time_diff[curr_waypoint];
            } else {
                // Last point
                q1 = t.waypoints[prev_waypoint][j];
                q2 = t.waypoints[curr_waypoint][j];
                q3 = q1;

                dt1 = dt2 = time_diff[curr_waypoint - 1];
            }

            double v1, v2, a;
            //bool start_velocity = false;

            if (dt1 == 0.0 || dt2 == 0.0) {
                v1 = 0.0;
                v2 = 0.0;
                a = 0.0;
            } else {
                // if (curr_waypoint == 0) {
                //   if (curr_waypoint->hasVelocities())
                //   {
                //     start_velocity = true;
                //     v1 = curr_waypoint->getVariableVelocity(idx[j]);
                //   }
                // }
                v1 = (q2 - q1) / dt1;
                v2 = (q3 - q2) / dt2;
                a = 2.0 * (v2 - v1) / (dt1 + dt2);
            }

            t.velocities[curr_waypoint][j] = (v1 + v2) / 2.0;
            t.accelerations[curr_waypoint][j] = a;
        }
    }
}

#endif
