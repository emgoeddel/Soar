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
