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

    object_data* o1_d = static_cast<object_data*>(o1->getUserData());
    object_data* o2_d = static_cast<object_data*>(o2->getUserData());

    if (o1_d->allowed_collisions.count(o2_d->name) ||
        o2_d->allowed_collisions.count(o1_d->name)) {
        return cd->done;
    }

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

    std::vector<fcl::CollisionObject*> obj_ptrs;
    std::vector<object_data*> obj_datas;
    for (std::map<std::string, transform3>::iterator t = xforms.begin();
         t != xforms.end(); t++) {
        vec4 quat;
        t->second.rotation(quat);
        fcl::Quaternion3f fcl_quat(quat[3], quat[0], quat[1], quat[2]);
        vec3 pos;
        t->second.position(pos);
        fcl::Vec3f fcl_vec(pos[0], pos[1], pos[2]);
        fcl::Transform3f fcl_xf(fcl_quat, fcl_vec);

        fcl::BVHModel<fcl::OBBRSS>* geom = &(model->all_links[t->first].collision_model);
        std::shared_ptr<fcl::CollisionGeometry> cg =
            std::shared_ptr<fcl::CollisionGeometry>(geom);

        obj_ptrs.push_back(new fcl::CollisionObject(cg, fcl_xf));
        obj_datas.push_back(new object_data());
        obj_datas.back()->name = t->first;
        obj_datas.back()->allowed_collisions = model->allowed[t->first];
        obj_ptrs.back()->setUserData(obj_datas.back());
        robot->registerObject(obj_ptrs.back());
    }


    collision_data cd;
    cd.request = fcl::CollisionRequest();
    cd.request.enable_contact = false;
    cd.request.enable_cost = false;
    cd.result = fcl::CollisionResult();
    robot->collide(&cd, collision_function);
    if (cd.result.isCollision()) {
        //std::cout << "Result: COLLISION" << std::endl;
        return false;
    }
    //std::cout << "Result: NO collision" << std::endl;
    return true;
}

#endif
