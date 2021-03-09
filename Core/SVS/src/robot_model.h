#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#ifdef ENABLE_ROS

#include <map>
#include <set>
#include "mat.h"

#include <fcl/BVH/BVH_model.h>
#include <kdl/tree.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

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
    ptlist vertices;
    fcl::BVHModel<fcl::OBBRSS> collision_model;
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
    robot_model();
    ~robot_model();

    bool init(std::string robot_desc);
    std::string robot_info();
    std::map<std::string, transform3> link_transforms(std::map<std::string, double> joints);
    std::vector<double> solve_ik(vec3 ee_pt);
    vec3 end_effector_pos(std::map<std::string, double> joints);

    std::string name;
    // root_link is assumed to be one of the links_of_interest
    std::string root_link;
    std::string default_joint_group;
    std::string end_effector;

    // link information
    std::map<std::string, link_info> all_links;
    std::set<std::string> links_of_interest;

    // joint information
    std::map<std::string, joint_info> all_joints;
    std::map<std::string, std::vector<std::string> > joint_groups;

    // allowed collisions
    std::map<std::string, std::set<std::string> > allowed;

private:
    void calculate_link_xform(std::string link_name,
                              std::map<std::string, double> pose,
                              std::map<std::string, transform3>& out);
    transform3 compose_joint_xform(std::string joint_name, double pos);
    std::vector<double> random_valid_pose(std::string group);

    KDL::Tree kin_tree;
    KDL::Chain ik_chain;
    KDL::ChainIkSolverPos_LMA* ik_solver;
    KDL::ChainFkSolverPos_recursive* fk_solver;
};

#endif
#endif
