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

class motor_state {
public:
    motor_state(std::shared_ptr<motor> m, std::string n);
    void copy_from(motor_state* other);

    //// Trajectory planning ////
    void new_query(int id, query q);
    std::string get_query_status(int id);
    int num_queries() { return queries.size(); }
    std::vector<int> get_query_ids();

    void search_started_callback(int id);
    void new_trajectory_callback(int id, trajectory t);
    void min_traj_callback(int id);
    void max_traj_callback(int id);
    int num_trajectories(int query_id);

    bool is_start_state_for(trajectory& t);

    //// Joint state tracking ////
    void set_joints(std::map<std::string, double> j);
    std::map<std::string, double> get_joints();
    bool has_joints();
    void set_joints_type(std::string jt);
    std::string get_joints_type() { return joints_type; }

    void set_base_xform(transform3 t);
    transform3 get_base_xform();

    std::map<std::string, transform3> get_link_transforms();

    std::string robot_name();

    //// Connection to motor_link ////
    void set_listener(motor_link* ml);
    void remove_listener();

    bool match_trajectory(int set_id, int traj_id, trajectory& out);

private:
    void notify_listener();

    std::shared_ptr<motor> mtr;

    std::string state_name;

    std::map<std::string, double> joints;
    std::mutex joints_mtx;
    std::string joints_type;

    transform3 base_xform;
    std::mutex xform_mtx;

    std::map<int, motor_query> queries;
    std::map<int, std::vector<trajectory> > trajectories;
    std::mutex traj_mtx;
    std::map<int, std::string> statuses;
    std::mutex stat_mtx;

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
    std::map<int, std::vector<Symbol*> > query_traj_map;
};

#endif
#endif
