#include "motor_state.h"
#include "motor.h"

motor_state::motor_state(motor* m, std::string n) : mtr(m),
                                                    model(mtr->get_model_ptr()),
                                                    state_name(n)
{
    base_xform = transform3::identity();
}

void motor_state::copy_from(motor_state* other) {
    // XXX This isn't quite right
    // How to deal with trajectories from parent state?
    trajectories = other->trajectories;

    std::lock_guard<std::mutex> guard1(joints_mtx);
    std::lock_guard<std::mutex> guard2(xform_mtx);
    joints = other->get_joints();
    base_xform = other->get_base_xform();
}

void motor_state::new_query(int id, query q) {
    motor_query mq;
    mq.soar_query = q;
    mq.start_state = get_joints();
    //mq.obstacles = get from scene;
    queries[id] = mq;
    if (!mtr->new_planner_query(id, mq, this)) {
        std::cout << "Warning: Could not start planning for query " << id << std::endl;
    }
}

void motor_state::new_trajectory_callback(int id, trajectory t) {
    trajectories[id].push_back(t);
    std::cout << "Added a trajectory to set with id " << id << std::endl;
}

void motor_state::set_joints(std::map<std::string, double> j) {
    std::lock_guard<std::mutex> guard(joints_mtx);

    for (std::map<std::string, double>::iterator i = j.begin();
         i != j.end(); i++) {
        joints[i->first] = i->second;
    }
}

std::map<std::string, double> motor_state::get_joints() {
    std::lock_guard<std::mutex> guard(joints_mtx);

    return joints;
}

bool motor_state::has_joints() {
    std::lock_guard<std::mutex> guard(joints_mtx);

    return (!joints.empty());
}

void motor_state::set_base_xform(transform3 t) {
    std::lock_guard<std::mutex> guard(xform_mtx);

    base_xform = t;
}

transform3 motor_state::get_base_xform() {
    std::lock_guard<std::mutex> guard(xform_mtx);

    return base_xform;
}

std::map<std::string, transform3> motor_state::get_link_transforms() {
    std::lock_guard<std::mutex> guard1(joints_mtx);
    std::lock_guard<std::mutex> guard2(xform_mtx);

    return model->link_transforms(joints);
}
