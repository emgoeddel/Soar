#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#ifdef ENABLE_ROS

#include <string>
#include <map>
#include <memory>
#include <mutex>

#include "mat.h"
#include "robot_model.h"
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

class motor_state {
public:
    motor_state(motor* m, std::string n);
    void copy_from(motor_state* other);

    //// Trajectory planning ////
    void new_query(int id, query q);

    void new_trajectory_callback(int id, trajectory t);
    //void search_finished_callback(int id);

    //// Joint state tracking ////
    void set_joints(std::map<std::string, double> j);
    std::map<std::string, double> get_joints();
    bool has_joints();

    void set_base_xform(transform3 t);
    transform3 get_base_xform();

    std::map<std::string, transform3> get_link_transforms();

    std::string robot_name() { return model->name; }

private:
    std::shared_ptr<motor> mtr;
    std::shared_ptr<robot_model> model;

    std::string state_name;

    std::map<std::string, double> joints;
    std::mutex joints_mtx;

    transform3 base_xform;
    std::mutex xform_mtx;

    std::map<int, motor_query> queries;
    std::map<int, std::vector<trajectory> > trajectories;
};

#endif
#endif
