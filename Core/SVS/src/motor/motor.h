#ifndef MOTOR_H
#define MOTOR_H

#ifdef ENABLE_ROS

#include <vector>
#include <mutex>

#include "robot_model.h"
#include "planning_problem.h"

/*
 * motor class
 *
 * Provides an interface to motion planning and robot kinematics
 *
 */

class collision_checker;

class motor {
public:
    motor(std::string urdf);
    ~motor();

    // Robot model and state queries
    std::string get_robot_name() { return model->get_robot_name(); }
    std::vector<std::string> get_link_names();
    std::map<std::string, vec3> get_link_boxes();
    std::map<std::string, transform3>
    get_link_transforms_at(std::map<std::string, double> j);
    transform3 get_ee_frame_transform_at(std::map<std::string, double> j);

    // Motion planning
    bool new_planner_query(int id, motor_query q, motor_state* msp);
    void stop_planner_query(int id);

    collision_checker* build_collision_checker(transform3 robot_base,
                                               std::map<std::string, double> pose,
                                               std::vector<obstacle>& obstacles);
    void check_collision_state(transform3 robot_base,
                               std::map<std::string, double> pose,
                               std::vector<obstacle>& obstacles);

private:
    std::shared_ptr<robot_model> model;
    std::vector<planning_problem*> ongoing;
};

#endif
#endif
