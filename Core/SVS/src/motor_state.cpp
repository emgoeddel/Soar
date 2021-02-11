#include "motor_state.h"
#include "motor.h"

motor_state::motor_state(motor* m, std::string n) : mtr(m),
                                                    model(mtr->get_model_ptr()),
                                                    state_name(n),
                                                    joints_type("none")
{
    base_xform = transform3::identity();
}

void motor_state::copy_from(motor_state* other) {
    joints_type = "copy";

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

std::vector<int> motor_state::get_query_ids() {
    std::vector<int> out;

    std::map<int, motor_query>::iterator i = queries.begin();
    for (; i != queries.end(); i++) {
        out.push_back(i->first);
    }

    return out;
}

void motor_state::new_trajectory_callback(int id, trajectory t) {
    trajectories[id].push_back(t);
    std::cout << "Added a trajectory to set with id " << id << std::endl;
    notify_listeners();
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

void motor_state::set_joints_type(std::string jt) {
    joints_type = jt;
    notify_listeners();
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

void motor_state::add_listener(motor_link* ml) {
    listeners.push_back(ml);
}

void motor_state::remove_listener(motor_link* ml) {
    listeners.remove(ml);
}

void motor_state::notify_listeners() {
    for (std::list<motor_link*>::iterator i = listeners.begin();
         i != listeners.end(); i++) {
        (*i)->update_desc();
    }
}

const std::string motor_link::joints_tag = "joint-state";
const std::string motor_link::type_tag = "type";
const std::string motor_link::traj_sets_tag = "trajectories";
const std::string motor_link::set_tag = "set";
const std::string motor_link::target_tag = "target";
const std::string motor_link::traj_tag = "trajectory";

motor_link::motor_link(soar_interface* si, Symbol* ln, motor_state* m)
    : ms(m), si(si), motor_sym(ln), joints_type("none")
{
    m->add_listener(this);

    state_sym = si->get_wme_val(si->make_id_wme(motor_sym, si->make_sym(joints_tag)));
    joints_type_wme = si->make_wme(state_sym, type_tag, joints_type);
    traj_sets_sym = si->get_wme_val(si->make_id_wme(motor_sym, si->make_sym(traj_sets_tag)));

    update_desc();
}

void motor_link::motor_link::update_desc() {
    // Update joint state information
    if (joints_type != ms->get_joints_type()) {
        joints_type = ms->get_joints_type();
        si->remove_wme(joints_type_wme);
        joints_type_wme = si->make_wme(state_sym,
                                       type_tag,
                                       joints_type);
    }

    // Update trajectory information
    std::vector<int> queries = ms->get_query_ids();
    for (std::vector<int>::iterator i = queries.begin(); i != queries.end(); i++) {
        if (query_sym_map.count(*i) == 0) {
            query_sym_map[*i] = si->get_wme_val(si->make_id_wme(traj_sets_sym,
                                                                si->make_sym(set_tag)));
        }

        if (ms->num_trajectories(*i) > query_traj_map[*i].size()) {
            query_traj_map[*i].push_back(si->make_id_wme(query_sym_map[*i],
                                                         si->make_sym(traj_tag)));
        }
    }
}
