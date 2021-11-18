#ifdef ENABLE_ROS

#include "motor.h"

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

bool motor::new_planner_query(int id, motor_query q, motor_state* msp) {
    ongoing.push_back(new planning_problem(id, q, msp, model));
    ongoing.back()->start_solve();
    return true;
}

#endif