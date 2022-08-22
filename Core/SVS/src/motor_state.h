#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#ifdef ENABLE_ROS

#include <memory>
#include <string>
#include <map>
#include <list>
#include <mutex>

#include "mat.h"
#include "soar_interface.h"
#include "motor_types.h"

/*
 * motor_state class
 *
 * Holds everything an individual SVS state needs to know for motion planning
 * Keeps track of the ongoing motion queries made in an SVS state and the
 * current set of trajectories that have been returned for those queries
 *
 */

class motor;
class motor_link;
class objective;
class scene;

class motor_state {
public:
    motor_state(std::shared_ptr<motor> m, scene* s, std::string n);
    void copy_from(motor_state* other);

    //// Trajectory planning ////
    void new_query(int id, query q);
    std::string get_query_status(int id);
    std::vector<int> get_query_failures(int id);
    void stop_query(int id);

    int num_queries() { return queries.size(); }
    std::vector<int> get_query_ids();
    bool has_query_id(int id); // refers to started queries
    bool has_set_id(int id); // refers to queries with actual trajectories

    void query_status_callback(int id, std::string stat);
    void new_trajectory_callback(int id, trajectory t);
    void failure_callback(int id, FailureType ft);
    int num_trajectories(int query_id);

    bool is_start_state_for(trajectory& t);
    void get_latest_trajectories(int set_id, std::map<int, trajectory>& out);
    std::map<int, double> trajectory_lengths(int id);

    //// Objective reporting ////
    void new_objective_callback(int id, objective* obj);
    void update_objective_callback(int id, std::string obj_name);

    bool has_objective_updates(int id);
    bool has_objective_updates(int id, std::string obj_name);
    void reset_objective_updates(int id, std::string obj_name);

    int num_objectives(int set_id);
    std::vector<std::string> objective_names(int set_id);
    objective* get_objective(int set_id, std::string name);

    //// Joint state tracking ////
    void set_joints(std::map<std::string, double> j);
    std::map<std::string, double> get_joints();
    transform3 ee_frame_for_joints();
    bool has_joints();
    void set_joints_type(std::string jt);
    std::string get_joints_type() { return joints_type; }

    void set_base_xform(transform3 t);
    transform3 get_base_xform();
    std::map<std::string, transform3> get_link_transforms();
    std::map<std::string, vec3> get_link_boxes();

    std::string robot_name();

    //// Connection to motor_link ////
    void set_listener(motor_link* ml);
    void remove_listener();

    bool match_trajectory(int set_id, int traj_id, trajectory& out);

private:
    void notify_listener();

    std::shared_ptr<motor> mtr;
    scene* scn;

    std::string state_name;

    std::map<std::string, double> joints;
    std::mutex joints_mtx;
    std::string joints_type;

    transform3 base_xform;
    std::mutex xform_mtx;

    std::map<int, motor_query> queries;
    std::map<int, std::vector<trajectory> > trajectories;
    std::mutex traj_mtx;
    std::map<int, std::map<std::string, objective*> > objectives;
    std::map<int, std::map<std::string, bool> > obj_has_update;
    std::mutex obj_mtx;
    std::map<int, std::string> statuses;
    std::mutex stat_mtx;
    std::map<int, std::vector<int> > failures;
    std::mutex fail_mtx;

    motor_link* listener;
};

/*
 * motor_link class
 *
 * Similar to a sgwme, this class is the interface between a state's
 * motor_state and the rest of Soar
 *
 */

class motor_link
{
public:
    motor_link(soar_interface* si, Symbol* ln, motor_state* m);
    ~motor_link();

    motor_state* get_motor_state() { return ms; }
    void update_desc();

    static const std::string joints_tag;
    static const std::string type_tag;
    static const std::string traj_sets_tag;
    static const std::string set_tag;
    static const std::string traj_count_tag;
    static const std::string target_tag;
    static const std::string traj_tag;
    static const std::string command_id_tag;
    static const std::string traj_id_tag;

private:
    motor_state* ms;

    soar_interface* si;
    Symbol* motor_sym;
    Symbol* state_sym;
    wme* joints_type_wme;
    Symbol* traj_sets_sym;
    std::vector<Symbol*> sets_syms;

    std::string joints_type;

    std::map<int, Symbol*> query_sym_map;
    std::map<int, wme*> query_count_map;
    std::map<int, std::map<int, Symbol*> > query_traj_map;
    std::map<int, std::map<std::string, std::map<int, wme*> > > query_obj_map;
};

#endif
#endif
