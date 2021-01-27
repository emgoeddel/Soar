#ifdef ENABLE_ROS
#include "robot_state.h"

robot_state::robot_state(robot_model* m) : model(m) {}

void robot_state::copy_from(robot_state* other) {
    joints = other->get_joints();
    base_xform = other->get_base_xform();
}

void robot_state::set_joints(std::map<std::string, double> j) {
    std::lock_guard<std::mutex> guard(joints_mtx);

    for (std::map<std::string, double>::iterator i = j.begin();
         i != j.end(); i++) {
        joints[i->first] = i->second;
    }
}

std::map<std::string, double> robot_state::get_joints() {
    std::lock_guard<std::mutex> guard(joints_mtx);

    return joints;
}

void robot_state::set_base_xform(transform3 t) {
    std::lock_guard<std::mutex> guard(xform_mtx);

    base_xform = t;
}

transform3 robot_state::get_base_xform() {
    std::lock_guard<std::mutex> guard(xform_mtx);

    return base_xform;
}

std::map<std::string, transform3> robot_state::get_link_transforms() {
    std::lock_guard<std::mutex> guard1(joints_mtx);
    std::lock_guard<std::mutex> guard2(xform_mtx);

    return model->link_transforms(joints);
}

#endif
