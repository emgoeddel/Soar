#ifdef ENABLE_ROS

#include "collision.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

bool collision_function(fcl::CollisionObject* o1,
                        fcl::CollisionObject* o2, void* cdata)
{
    collision_data* cd = static_cast<collision_data*>(cdata);
    const fcl::CollisionRequest& request = cd->request;
    fcl::CollisionResult& result = cd->result;

    if(cd->done) return true;

    fcl::collide(o1, o2, request, result);

    if(!request.enable_cost &&
       (result.isCollision()) &&
       (result.numContacts() >= request.num_max_contacts))
        cd->done = true;

    return cd->done;
}

collision_checker::collision_checker(ompl::base::SpaceInformation* si)
    : ompl::base::StateValidityChecker(si)
{}

collision_checker::collision_checker(const ompl::base::SpaceInformationPtr& si)
    : ompl::base::StateValidityChecker(si)
{}

collision_checker::collision_checker(ompl::base::SpaceInformation* si,
                                     robot_model* m,
                                     std::string group)
    : ompl::base::StateValidityChecker(si),
    model(m)
{
    joint_names = model->joint_groups[group];

    robot = new fcl::DynamicAABBTreeCollisionManager();
    world = new fcl::DynamicAABBTreeCollisionManager();
}

collision_checker::collision_checker(const ompl::base::SpaceInformationPtr& si,
                                     robot_model* m,
                                     std::string group)
    : ompl::base::StateValidityChecker(si),
    model(m)
{
    joint_names = model->joint_groups[group];

    robot = new fcl::DynamicAABBTreeCollisionManager();
    world = new fcl::DynamicAABBTreeCollisionManager();
}

bool collision_checker::isValid(const ompl::base::State* state) const {
    robot->clear();

    // cast the abstract state type to the type we expect
    const ompl::base::RealVectorStateSpace::StateType* vecstate =
        state->as<ompl::base::RealVectorStateSpace::StateType>();

    std::map<std::string, double> joint_state;
    for (int i = 0; i < joint_names.size(); i++) {
        joint_state[joint_names[i]] = (*vecstate)[i];
    }
    std::map<std::string, transform3> xforms = model->link_transforms(joint_state);

    for (std::map<std::string, transform3>::iterator t = xforms.begin();
         t != xforms.end(); t++) {
        fcl::CollisionObject* link_obj;

        vec4 quat;
        t->second.rotation(quat);
        fcl::Quaternion3f fcl_quat(quat[3], quat[0], quat[1], quat[2]);
        vec3 pos;
        t->second.position(pos);
        fcl::Vec3f fcl_vec(pos[0], pos[1], pos[3]);
        fcl::Transform3f fcl_xf(fcl_quat, fcl_vec);

        fcl::BVHModel<fcl::OBBRSS> geom = model->all_links[t->first].collision_model;
        std::shared_ptr<fcl::CollisionGeometry> cg =
            std::shared_ptr<fcl::CollisionGeometry>(&geom);

        link_obj = new fcl::CollisionObject(cg, fcl_xf);
        robot->registerObject(link_obj);
    }

    collision_data cd;
    cd.request = fcl::CollisionRequest();
    cd.request.enable_contact = false;
    cd.request.enable_cost = false;
    cd.result = fcl::CollisionResult();
    robot->collide(&cd, collision_function);
    if (cd.result.isCollision()) return false;
    return true;
}

#endif
