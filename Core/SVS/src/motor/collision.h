#ifndef COLLISION_H
#define COLLISION_H

#ifdef ENABLE_ROS

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#include "robot_model.h"
#include "motor_types.h"

/*
 * collision_data struct
 *
 */

struct collision_data {
    collision_data() {
        done = false;
    }

    fcl::CollisionRequest request;
    fcl::CollisionResult result;
    bool done;
};

bool collision_function(fcl::CollisionObject* o1,
                        fcl::CollisionObject* o2, void* cdata);

struct distance_data {
    distance_data() {
        done = false;
    }

    fcl::DistanceRequest request;
    fcl::DistanceResult result;
    bool done;
};

// bool distance_function(fcl::CollisionObject* o1,
//                        fcl::CollisionObject* o2, void* ddata);
bool distance_function(fcl::CollisionObject* o1,
                       fcl::CollisionObject* o2,
                       void* ddata, fcl::FCL_REAL& dist);

/*
 * object_data struct
 *
 * To pass in as user data for each collision object
 */
struct object_data {
    std::string name;
    std::set<std::string> allowed_collisions;
};

class collision_checker : public ompl::base::StateValidityChecker {
public:
    collision_checker(ompl::base::SpaceInformation* si);
    collision_checker(const ompl::base::SpaceInformationPtr& si);
    collision_checker(ompl::base::SpaceInformation* si,
                      std::shared_ptr<robot_model> m,
                      transform3 rb,
                      std::string group,
                      std::map<std::string, double> fixed,
                      std::vector<obstacle>& obstacles);
    collision_checker(const ompl::base::SpaceInformationPtr& si,
                      std::shared_ptr<robot_model> m,
                      transform3 rb,
                      std::string group,
                      std::map<std::string, double> fixed,
                      std::vector<obstacle>& obstacles);
    ~collision_checker();

    bool isValid(const ompl::base::State* state) const override;
    void print_scene(const ompl::base::State* state);
    double minimum_distance(std::map<std::string, double> state);

    ompl::base::StateSpacePtr get_space() { return si_->getStateSpace(); }

private:
    void setup_obstacles(std::vector<obstacle>& obstacles);

    std::vector<std::string> joint_names;
    std::map<std::string, double> fixed_joints;
    std::shared_ptr<robot_model> model;
    transform3 robot_base;

    fcl::BroadPhaseCollisionManager* world;
    std::vector<fcl::CollisionObject*> world_objects;
    std::vector<std::shared_ptr<fcl::CollisionGeometry> > world_obj_geoms;
    std::vector<ObstacleType> geom_types;
    std::vector<object_data*> world_obj_datas;
    bool world_ready;
};

#endif
#endif
