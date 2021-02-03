#ifndef MOTOR_H
#define MOTOR_H

#ifdef ENABLE_ROS

#include <map>
#include <memory>
#include <mutex>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/collision.h>

#include "mat.h"
#include "trajectory.h"
#include "robot_model.h"

/*
 * collision_data struct
 *
 * Why doesn't FCL provide this?
 */

struct collision_data
{
    collision_data()
    {
        done = false;
    }

    fcl::CollisionRequest request;
    fcl::CollisionResult result;
    bool done;
};

// Same question as above!
bool collision_function(fcl::CollisionObject* o1,
                        fcl::CollisionObject* o2, void* cdata);

/*
 * planning_problem class
 *
 * Holds the search space and collision info for a particular planning
 * query and runs the search through an OMPL SimpleSetup with FCL collision
 *
 */

class planning_problem {
public:
    planning_problem(int qid, motor_query q, robot_model* m);

private:
    bool state_valid(const ompl::base::State* state);

    int query_id;
    motor_query query;
    std::vector<std::string> joint_names;

    std::shared_ptr<robot_model> model;

    ompl::geometric::SimpleSetup* ompl_ss;

    fcl::BroadPhaseCollisionManager* robot;
    fcl::BroadPhaseCollisionManager* world;
};

/*
 * motor class
 *
 * Provides an interface to motion planning and robot kinematics
 *
 */

class motor {
public:
    motor(std::string urdf);

    robot_model* get_model_ptr() { return &model; }
    std::vector<std::string> get_link_names();

    std::string robot_name() { return model.name; }

    bool new_planner_query(int id, motor_query q);

private:
    robot_model model;

    std::vector<planning_problem> ongoing;
};

#endif
#endif
