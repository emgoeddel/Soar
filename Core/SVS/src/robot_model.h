#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#ifdef ENABLE_ROS

#include <map>
#include <urdf/model.h>
//#include <urdf_model/model.h>

#include "mat.h"

enum joint_type {
    REVOLUTE,
    CONTINUOUS,
    PRISMATIC,
    FIXED,
    UNSUPPORTED
};

struct joint_info {
    std::string name;

    std::string parent_link;
    std::string child_link;

    joint_type type;
    transform3 origin;
    vec3 axis;

    double max_pos;
    double min_pos;
    double max_velocity;
    double max_effort;
};

struct link_info {
    std::string name;

    std::string parent_joint;
    std::set<std::string> child_joints;

    transform3 collision_origin;
    std::string mesh_file;
};

/*
 * robot_model class
 *
 * Imports and stores necessary information about links and joints
 * from URDF, provides FK
 *
 */

class robot_model {
public:
    robot_model() : name("none") {};

    bool init(std::string robot_desc);
    std::string robot_info();
    std::map<std::string, transform3> link_transforms(std::map<std::string, double> joints);

    std::string name;
    // root_link is assumed to be one of the links_of_interest
    std::string root_link;
    std::string default_joint_group;

    // link information
    std::map<std::string, link_info> all_links;
    std::set<std::string> links_of_interest;

    // joint information
    std::map<std::string, joint_info> all_joints;
    std::map<std::string, std::vector<std::string> > joint_groups;

private:
    void calculate_link_xform(std::string link_name,
                              std::map<std::string, double> pose,
                              std::map<std::string, transform3>& out);
    transform3 compose_joint_xform(std::string joint_name, double pos);
};

#endif
#endif
