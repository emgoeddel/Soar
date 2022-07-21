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

void motor::check_collision_state(transform3 robot_base,
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
            bounds.setLow(b, -10*M_PI);
            bounds.setHigh(b, 10*M_PI);
        }
        b++;
    }

    bounds.check();
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    space->setLongestValidSegmentFraction(0.005);

    ompl::base::SpaceInformation si(space);

    collision_checker cc(&si, model, robot_base, "arm", obstacles);

    ompl::base::ScopedState<> ompl_state(space);
    int c = 0;
    for (std::vector<std::string>::iterator j = joints.begin(); j != joints.end(); j++)
    {
        ompl_state[c] = pose[*j];
        c++;
    }

    cc.print_scene(ompl_state.get());
}

#endif
