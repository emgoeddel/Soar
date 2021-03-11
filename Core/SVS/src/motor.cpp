#ifdef ENABLE_ROS

#include "motor.h"

motor::motor(std::string urdf) {
    model.init(urdf);
    std::cout << model.robot_info();
}

motor::~motor() {
    for (std::vector<planning_problem*>::iterator i = ongoing.begin();
         i != ongoing.end(); i++) {
        delete *i;
    }
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
    ongoing.push_back(new planning_problem(id, q, msp, &model));
    ongoing.back()->find_one();
    return true;
}

#endif
