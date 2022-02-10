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
    world(NULL)
{}

collision_checker::collision_checker(const ompl::base::SpaceInformationPtr& si)
    : ompl::base::StateValidityChecker(si),
    world(NULL)
{}

collision_checker::collision_checker(ompl::base::SpaceInformation* si,
                                     std::shared_ptr<robot_model> m,
                                     transform3 rb,
                                     std::string group,
                                     std::vector<obstacle>& obstacles)
    : ompl::base::StateValidityChecker(si),
    robot_base(rb),
    model(m)
{
    joint_names = model->get_joint_group(group);

    world = new fcl::DynamicAABBTreeCollisionManager();

    setup_obstacles(obstacles);
}

collision_checker::collision_checker(const ompl::base::SpaceInformationPtr& si,
                                     std::shared_ptr<robot_model> m,
                                     transform3 rb,
                                     std::string group,
                                     std::vector<obstacle>& obstacles)
    : ompl::base::StateValidityChecker(si),
    robot_base(rb),
    model(m)
{
    joint_names = model->get_joint_group(group);

    world = new fcl::DynamicAABBTreeCollisionManager();

    setup_obstacles(obstacles);
}

collision_checker::~collision_checker() {
    std::vector<fcl::CollisionObject*>::iterator o = world_objects.begin();
    for (; o != world_objects.end(); o++) {
        delete *o;
    }
    std::vector<object_data*>::iterator d = world_obj_datas.begin();
    for (; d != world_obj_datas.end(); d++) {
        delete *d;
    }

    if (world) delete world;
}

// Non-robot obstacles are set up once per planning problem
void collision_checker::setup_obstacles(std::vector<obstacle>& obstacles) {
    world->clear();

    std::vector<obstacle>::iterator i = obstacles.begin();
    for (; i != obstacles.end(); i++) {
        transform3 t(i->translation, i->rotation, i->scale);

        vec4 quat;
        t.rotation(quat);
        fcl::Quaternion3f fcl_quat(quat[3], quat[0], quat[1], quat[2]);

        fcl::Vec3f fcl_vec(i->translation[0],
                           i->translation[1],
                           i->translation[2]);

        fcl::Transform3f fcl_xf(fcl_quat, fcl_vec);

        if (i->geometry == BALL_OBSTACLE) {
            world_obj_geoms.push_back(std::shared_ptr<fcl::CollisionGeometry>(new fcl::Sphere(i->ball_radius)));
        } else if (i->geometry == BOX_OBSTACLE) {
            world_obj_geoms.push_back(std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(i->box_dim[0], i->box_dim[1], i->box_dim[2])));
        } else if (i->geometry == CONVEX_OBSTACLE) {
            std::cout << "Warning: Collision detection for convex obstacle "
                      << i->name << " not supported" << std::endl;
            continue;
            // XXX FCL does NOT support objects made only of vertices, even though it
            // seems like the following should work from documentation
            // std::vector<fcl::Vec3f> points(i->convex_pts.size());
            // for (int v = 0; v < i->convex_pts.size(); v++) {
            //     points[v] = fcl::Vec3f(i->convex_pts[v][0],
            //                            i->convex_pts[v][1],
            //                            i->convex_pts[v][2]);
            // }
            // fcl::BVHModel<fcl::OBBRSS>* mdl = new fcl::BVHModel<fcl::OBBRSS>();
            // mdl->beginModel();
            // mdl->addSubModel(points);
            // mdl->endModel();
            // world_obj_geoms.push_back(std::shared_ptr<fcl::CollisionGeometry>(mdl));
        } else continue; // NON_OBSTACLE shouldn't even get here

        world_objects.push_back(new fcl::CollisionObject(world_obj_geoms.back(),
                                                         fcl_xf));

        world_obj_datas.push_back(new object_data());
        world_obj_datas.back()->name = i->name;
        world_objects.back()->setUserData(world_obj_datas.back());

        world->registerObject(world_objects.back());
    }
}

bool collision_checker::isValid(const ompl::base::State* state) const {
    fcl::BroadPhaseCollisionManager* robot = new fcl::DynamicAABBTreeCollisionManager();

    // cast the abstract state type to the type we expect
    const ompl::base::RealVectorStateSpace::StateType* vecstate =
        state->as<ompl::base::RealVectorStateSpace::StateType>();

    // Robot parts are updated for every state
    std::map<std::string, double> joint_state;
    for (int i = 0; i < joint_names.size(); i++) {
        joint_state[joint_names[i]] = (*vecstate)[i];
    }
    // Asking for the transforms FOR THE MESH MODELS FOR COLLISION
    std::map<std::string, transform3> xforms = model->link_transforms(joint_state, false);

    std::vector<fcl::CollisionObject*> obj_ptrs;
    std::vector<object_data*> obj_datas;
    for (std::map<std::string, transform3>::iterator t = xforms.begin();
         t != xforms.end(); t++) {
        transform3 wx = robot_base*t->second;

        vec4 quat;
        wx.rotation(quat);
        fcl::Quaternion3f fcl_quat(quat[3], quat[0], quat[1], quat[2]);
        vec3 pos;
        wx.position(pos);
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

    // Self-collision
    collision_data cds;
    cds.request = fcl::CollisionRequest();
    cds.request.enable_contact = false;
    cds.request.enable_cost = false;
    cds.result = fcl::CollisionResult();
    robot->collide(&cds, collision_function);

    // World collision
    collision_data cdw;
    cdw.request = fcl::CollisionRequest();
    cdw.request.enable_contact = false;
    cdw.request.enable_cost = false;
    cdw.result = fcl::CollisionResult();
    robot->collide(world, &cdw, collision_function);

    std::vector<fcl::CollisionObject*>::iterator o = obj_ptrs.begin();
    for (; o != obj_ptrs.end(); o++) {
        delete *o;
    }
    std::vector<object_data*>::iterator d = obj_datas.begin();
    for (; d != obj_datas.end(); d++) {
        delete *d;
    }
    obj_datas.clear();

    delete robot;

    if (cds.result.isCollision() || cdw.result.isCollision())
        return false;

    return true;
}

#endif
