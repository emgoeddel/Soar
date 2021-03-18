#include "motor_state.h"
#include <math.h>
#include "motor/motor.h"

motor_state::motor_state(std::shared_ptr<motor> m, std::string n) : mtr(m),
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
    std::cout << "Added a trajectory of length " << trajectories[id].back().length
              << " to set with id " << id << std::endl;
    notify_listener();
}

bool motor_state::is_start_state_for(trajectory& t) {
    std::lock_guard<std::mutex> guard(joints_mtx);

    std::vector<double>& wp = t.waypoints[0];

    std::vector<std::string>::iterator i = t.joints.begin();
    int j_ind = 0;
    for (; i != t.joints.end(); i++) {
        // XXX Fix magic number in this threshold
        if (fabs(wp[j_ind] - joints[*i]) > 0.01) {
            return false;
        }
        j_ind++;
    }

    return true;
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
    notify_listener();
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

    return mtr->get_link_transforms_at(joints);
}

std::string motor_state::robot_name() {
    return mtr->get_robot_name();
}

void motor_state::set_listener(motor_link* ml) {
    listener = ml;
}

void motor_state::remove_listener() {
    listener = NULL;
}

bool motor_state::match_trajectory(int set_id, int traj_id, trajectory& out) {
    if (trajectories.count(set_id) == 0) {
        std::cout << "Error: Motor state does not have a set with id " << set_id
                  << std::endl;
        return false;
    }

    if (trajectories[set_id].size() <= traj_id) {
        std::cout << "Error: Motor state does not have a trajectory in set " << set_id
                  << " with id " << traj_id << std::endl;
        return false;
    }

    std::cout << "Found a matching trajectory with length "
              << trajectories[set_id][traj_id].length
              << std::endl;
    out = trajectories[set_id][traj_id];
    return true;
}

void motor_state::notify_listener() {
    listener->update_desc();
}

const std::string motor_link::joints_tag = "joint-state";
const std::string motor_link::type_tag = "type";
const std::string motor_link::traj_sets_tag = "trajectories";
const std::string motor_link::set_tag = "set";
const std::string motor_link::target_tag = "target";
const std::string motor_link::traj_tag = "trajectory";
const std::string motor_link::command_id_tag = "command-id";
const std::string motor_link::traj_id_tag = "id";

motor_link::motor_link(soar_interface* si, Symbol* ln, motor_state* m)
    : ms(m), si(si), motor_sym(ln), joints_type("none")
{
    m->set_listener(this);

    state_sym = si->get_wme_val(si->make_id_wme(motor_sym, joints_tag));
    joints_type_wme = si->make_wme(state_sym, type_tag, joints_type);
    traj_sets_sym = si->get_wme_val(si->make_id_wme(motor_sym, traj_sets_tag));

    update_desc();
}


motor_link::~motor_link() {}

void motor_link::update_desc() {
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
            si->make_wme(query_sym_map[*i], command_id_tag, si->make_sym(*i));
            query_traj_map[*i] = std::vector<Symbol*>();
        }

        // XXX Assumes only one trajectory added per cycle, need to fix!
        int curr_num_traj = ms->num_trajectories(*i);
        if (curr_num_traj > query_traj_map[*i].size()) {
            query_traj_map[*i].push_back(
                si->get_wme_val(si->make_id_wme(query_sym_map[*i],
                                                si->make_sym(traj_tag))));
            si->make_wme(query_traj_map[*i].back(),
                         traj_id_tag,
                         si->make_sym(curr_num_traj - 1)); // Zero-indexed id
        }
    }
}
