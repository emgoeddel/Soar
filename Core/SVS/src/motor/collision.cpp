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
    : ompl::base::StateValidityChecker(si),
    robot(NULL),
    world(NULL)
{}

collision_checker::collision_checker(const ompl::base::SpaceInformationPtr& si)
    : ompl::base::StateValidityChecker(si),
    robot(NULL),
    world(NULL)
{}

collision_checker::collision_checker(ompl::base::SpaceInformation* si,
                                     std::shared_ptr<robot_model> m,
                                     std::string group,
                                     std::vector<obstacle>& obstacles)
    : ompl::base::StateValidityChecker(si),
    model(m)
{
    joint_names = model->get_joint_group(group);

    robot = new fcl::DynamicAABBTreeCollisionManager();
    world = new fcl::DynamicAABBTreeCollisionManager();

    setup_obstacles(obstacles);
}

collision_checker::collision_checker(const ompl::base::SpaceInformationPtr& si,
                                     std::shared_ptr<robot_model> m,
                                     std::string group,
                                     std::vector<obstacle>& obstacles)
    : ompl::base::StateValidityChecker(si),
    model(m)
{
    joint_names = model->get_joint_group(group);

    robot = new fcl::DynamicAABBTreeCollisionManager();
    world = new fcl::DynamicAABBTreeCollisionManager();

    setup_obstacles(obstacles);
}

collision_checker::~collision_checker() {
    if (robot) delete robot;
    if (world) delete world;
}

// Non-robot obstacles are set up once per planning problem
void collision_checker::setup_obstacles(std::vector<obstacle>& obstacles) {
    std::vector<obstacle>::iterator i = obstacles.begin();
    for (; i != obstacles.end(); i++) {
        transform3 t(i->translation, i->rotation, i->scale);

        vec4 quat;
        t.rotation(quat);
        fcl::Quaternion3f fcl_quat(quat[3], quat[0], quat[1], quat[2]);

        vec3 pos;
        i->translation;
        fcl::Vec3f fcl_vec(pos[0], pos[1], pos[2]);

        fcl::Transform3f fcl_xf(fcl_quat, fcl_vec);

        world_obj_geoms.push_back(std::shared_ptr<fcl::CollisionGeometry>(new fcl::Sphere(0)));

        world_objects.push_back(new fcl::CollisionObject(world_obj_geoms.back(),
                                                         fcl_xf));

        world_obj_datas.push_back(new object_data());
        world_obj_datas.back()->name = i->name;
        world_objects.back()->setUserData(world_obj_datas.back());

        world->registerObject(world_objects.back());
    }
}

bool collision_checker::isValid(const ompl::base::State* state) const {
    robot->clear();

    // cast the abstract state type to the type we expect
    const ompl::base::RealVectorStateSpace::StateType* vecstate =
        state->as<ompl::base::RealVectorStateSpace::StateType>();

    // Robot parts are updated for every state
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

        obj_ptrs.push_back(new fcl::CollisionObject(model->get_collision_model(t->first),
                                                    fcl_xf));
        obj_datas.push_back(new object_data());
        obj_datas.back()->name = t->first;
        obj_datas.back()->allowed_collisions = model->get_allowed_collisions(t->first);
        obj_ptrs.back()->setUserData(obj_datas.back());
        robot->registerObject(obj_ptrs.back());
    }

    collision_data cd;
    cd.request = fcl::CollisionRequest();
    cd.request.enable_contact = false;
    cd.request.enable_cost = false;
    cd.result = fcl::CollisionResult();
    robot->collide(&cd, collision_function);

    std::vector<fcl::CollisionObject*>::iterator o = obj_ptrs.begin();
    for (; o != obj_ptrs.end(); o++) {
        delete *o;
    }
    std::vector<object_data*>::iterator d = obj_datas.begin();
    for (; d != obj_datas.end(); d++) {
        delete *d;
    }
    obj_datas.clear();

    if (cd.result.isCollision()) {
        //std::cout << "Result: COLLISION" << std::endl;
        return false;
    }
    //std::cout << "Result: NO collision" << std::endl;
    return true;
}

#endif
