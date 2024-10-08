#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#ifdef ENABLE_ROS

#include <map>
#include <set>
#include <memory>
#include <mutex>
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
    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS> > collision_model;
    transform3 aabb_origin;
    vec3 aabb_size;
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

    // Data access
    std::string robot_info();

    std::string get_robot_name();

    std::string get_default_joint_group();

    std::vector<std::string> get_joint_group(std::string grp_name);
    int get_joint_group_size(std::string grp_name);

    joint_type get_joint_type(std::string joint_name);
    double get_joint_min(std::string joint_name);
    double get_joint_max(std::string joint_name);
    double get_joint_max_velocity(std::string joint_name);
    double get_joint_max_effort(std::string joint_name);

    std::set<std::string> get_links_of_interest();

    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS> > get_collision_model(std::string link_name);
    std::set<std::string> get_allowed_collisions(std::string link_name);
    std::map<std::string, vec3> models_as_boxes();

    // Kinematics
    // All input/output transforms are expected to be relative to the robot's BASE
    std::map<std::string, transform3> link_transforms(std::map<std::string, double> joints,
                                                      bool box_translation);
    std::vector<double> solve_ik(vec3 ee_pt, double torso_jnt);
    std::vector<double> solve_ik(vec3 ee_pt, vec3 ee_rot, double torso_jnt);
    std::vector<double> solve_ik_from(vec3 ee_pt, vec3 ee_rot,
                                      std::map<std::string, double> joints);
    vec3 end_effector_pos(std::map<std::string, double> joints);
    vec4 end_effector_rot(std::map<std::string, double> joints);
    transform3 end_effector_xform(std::map<std::string, double> joints);
    std::set<std::string> end_effector_links();

private:
    transform3 torso_xform(double pos);
    void calculate_link_xform(std::string link_name,
                              std::map<std::string, double> pose,
                              std::map<std::string, transform3>& out);
    transform3 compose_joint_xform(std::string joint_name, double pos);
    std::vector<double> random_valid_pose(std::string group);
    void solve_fk_internal(std::map<std::string, double> jnt_in, KDL::Frame& f);
    void solve_ik_internal(KDL::Frame f, bool rpy, std::vector<double>& jnt_out);

    bool initialized;

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

    KDL::Tree kin_tree;
    KDL::Chain base_chain;
    KDL::Chain torso_chain;
    std::mutex ik_mtx;
    KDL::ChainIkSolverPos_LMA* ik_solver_xyz;
    KDL::ChainIkSolverPos_LMA* ik_solver_xyzrpy;
    std::mutex fk_mtx;
    KDL::ChainFkSolverPos_recursive* fk_solver;
};

#endif
#endif
