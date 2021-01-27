#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#ifdef ENABLE_ROS

#include <string>
#include <map>
#include <memory>
#include <mutex>

#include "mat.h"
#include "robot_model.h"

class robot_state {
public:
    robot_state(std::shared_ptr<robot_model> m);

    void set_joints(std::map<std::string, double> j);
    std::map<std::string, double> get_joints();

    void set_base_xform(transform3 t);
    transform3 get_base_xform();

    std::map<std::string, transform3> get_link_transforms();

private:
    std::map<std::string, double> joints;
    std::mutex joints_mtx;

    transform3 base_xform;
    std::mutex xform_mtx;

    std::shared_ptr<robot_model> model;
};

#endif
#endif
