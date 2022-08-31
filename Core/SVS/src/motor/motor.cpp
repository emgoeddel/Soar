#ifdef ENABLE_ROS

#include "motor.h"
#include "collision.h"

motor::motor(std::string urdf) {
    model = std::make_shared<robot_model>();
    model->init(urdf);
    std::cout << model->robot_info();
}

motor::~motor() {
    for (std::vector<planning_problem*>::iterator i = ongoing.begin();
         i != ongoing.end(); i++) {
        delete *i;
    }
}

std::vector<std::string> motor::get_link_names() {
    std::vector<std::string> link_names;
    std::set<std::string> loi = model->get_links_of_interest();

    for (std::set<std::string>::iterator i = loi.begin(); i != loi.end(); i++) {
        link_names.push_back(*i);
    }

    return link_names;
}

std::map<std::string, vec3> motor::get_link_boxes() {
    return model->models_as_boxes();
}

std::map<std::string, transform3>
motor::get_link_transforms_at(std::map<std::string, double> j) {
    // Asking for the transforms FOR THE BOX MODELS FOR SVS
    // //std::cout << "SVS joint state: " << std::endl;
    // for (std::map<std::string, double>::iterator js = j.begin();
    //      js != j.end(); js++) {
    //     std::cout << "   " << js->first << ": " << js->second << std::endl;
    // }
    return model->link_transforms(j, true);
}

transform3 motor::get_ee_frame_transform_at(std::map<std::string, double> j) {
    // Asking for the tranform of the GRIPPER FRAME (NOT BOX) FROM KDL
    return model->end_effector_xform(j);
}

bool motor::new_planner_query(int id, motor_query q, motor_state* msp) {
    ongoing.push_back(new planning_problem(id, q, msp, model));
    ongoing.back()->start_solve();
    return true;
}

void motor::stop_planner_query(int id) {
    std::vector<planning_problem*>::iterator i = ongoing.begin();
    bool found = false;
    for (; i != ongoing.end(); i++) {
        if ((*i)->get_id() == id) {
            (*i)->stop_solve();
            found = true;
            break;
        }
    }
    if (!found) std::cout << "[WARNING] Attempting to stop a non-existent query!"
                          << std::endl;
}

bool motor::plan_straight_line(std::map<std::string, double> start,
                               vec3 goal_xyz, trajectory& out) {
    std::cout << "Planning straight line!" << std::endl;
    vec3 start_xyz = model->end_effector_pos(start);
    vec3 rpy = model->end_effector_rot(start);

    // Calculate amount of desired hand movement per IK step
    double move_len = sqrt(pow(goal_xyz.x() - start_xyz.x(), 2) +
                           pow(goal_xyz.y() - start_xyz.y(), 2) +
                           pow(goal_xyz.z() - start_xyz.z(), 2));
    vec3 move_unit((goal_xyz.x() - start_xyz.x()) / move_len,
                   (goal_xyz.y() - start_xyz.y()) / move_len,
                   (goal_xyz.z() - start_xyz.z()) / move_len);
    double STEP_LENGTH = 0.005; // 0.5 cm
    vec3 step_vector(move_unit.x() * STEP_LENGTH,
                     move_unit.y() * STEP_LENGTH,
                     move_unit.z() * STEP_LENGTH);

    // start setting up trajectory
    out.joints = model->get_joint_group("arm");
    out.fixed_joints["torso_lift_joint"] = start["torso_lift_joint"];
    out.fixed_joints["l_gripper_finger_joint"] = start["l_gripper_finger_joint"];
    out.fixed_joints["r_gripper_finger_joint"] = start["r_gripper_finger_joint"];
    std::cout << "Fixed values are " << out.fixed_joints["torso_lift_joint"]
              << ", " << out.fixed_joints["l_gripper_finger_joint"]
              << ", " << out.fixed_joints["r_gripper_finger_joint"] << std::endl;
    out.waypoints.clear();
    out.waypoints.push_back(std::vector<double>());
    std::vector<std::string>::iterator j = out.joints.begin();
    for (; j != out.joints.end(); j++) {
        out.waypoints[0].push_back(start[*j]);
    }

    // Use IK to take repeated steps
    double dist = move_len;
    vec3 cur_xyz = start_xyz;
    std::map<std::string, double> prev_joints = start;
    int step_num = 0;
    while (dist > STEP_LENGTH) {
        cur_xyz += step_vector;
        std::vector<double> new_joints = model->solve_ik_from(cur_xyz, rpy, prev_joints);

        if (new_joints.size() == 0) {
            std::cout << "IK FAIL at step " << step_num << std::endl;
            out.waypoints.clear();
            return false;
        }

        out.waypoints.push_back(new_joints);

        int ji = 0;
        for (j = out.joints.begin(); j != out.joints.end(); j++) {
            prev_joints[*j] = new_joints[ji];
            ji++;
        }

        dist -= STEP_LENGTH;
        step_num++;
        std::cout << "Dist remaining? " << dist << std::endl;
    }

    // Final IK step
    std::vector<double> last_joints = model->solve_ik_from(goal_xyz, rpy, prev_joints);

    if (last_joints.size() == 0) {
        std::cout << "IK FAIL at final step!" << std::endl;
        out.waypoints.clear();
        return false;
    }

    out.waypoints.push_back(last_joints);
    out.length = out.waypoints.size();

    double cur_time = 0.0;
    for (int w = 0; w < out.waypoints.size(); w++) {
        out.times.push_back(cur_time);
        cur_time += 0.2; // XXX This is a hack
    }

    return true;
}

collision_checker* motor::build_collision_checker(transform3 robot_base,
                                                  std::map<std::string, double> pose,
                                                  std::vector<obstacle>& obstacles) {
    // construct vector state space for arm
    std::vector<std::string> joints = model->get_joint_group("arm");
    int dof = joints.size();
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(dof));

    // set the bounds for state space based on joint limits
    ompl::base::RealVectorBounds bounds(dof);
    int b = 0;
    for (std::vector<std::string>::iterator i = joints.begin();
         i != joints.end(); i++)
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
    space->setLongestValidSegmentFraction(0.005);

    std::map<std::string, double> fixed;
    std::map<std::string, double>::iterator i = pose.begin();
    for (; i != pose.end(); i++) {
        bool is_fixed = true;
        std::vector<std::string>::iterator j = joints.begin();
        for (; j != joints.end(); j++) {
            if (*j == i->first) {
                is_fixed = false;
                break;
            }
        }
        if (is_fixed) fixed[i->first] = i->second;
    }

    return new collision_checker(new ompl::base::SpaceInformation(space), model,
                                 robot_base, "arm", fixed, obstacles);
}

void motor::check_collision_state(transform3 robot_base,
                                  std::map<std::string, double> pose,
                                  std::vector<obstacle>& obstacles) {
    collision_checker* cc = build_collision_checker(robot_base, pose, obstacles);

    ompl::base::ScopedState<> ompl_state(cc->get_space());
    std::vector<std::string> joints = model->get_joint_group("arm");
    int c = 0;
    for (std::vector<std::string>::iterator j = joints.begin(); j != joints.end(); j++)
    {
        ompl_state[c] = pose[*j];
        c++;
    }

    cc->print_scene(ompl_state.get());
    delete cc;
}

#endif
