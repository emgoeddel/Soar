#ifdef ENABLE_ROS

#include "collision.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

bool collision_function(fcl::CollisionObject* o1,
                        fcl::CollisionObject* o2, void* cdata) {
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

// bool distance_function(fcl::CollisionObject* o1,
//                        fcl::CollisionObject* o2, void* ddata) {
//     distance_data* dd = static_cast<distance_data*>(ddata);
//     const fcl::DistanceRequest& request = dd->request;
//     fcl::DistanceResult& result = dd->result;

//     if(dd->done) return true;

//     fcl::distance(o1, o2, request, result);

//     if (result.min_distance <= 0) return true;
//     return dd->done;
// }

bool distance_function(fcl::CollisionObject* o1,
                       fcl::CollisionObject* o2,
                       void* ddata, fcl::FCL_REAL& dist) {
    distance_data* dd = static_cast<distance_data*>(ddata);
    const fcl::DistanceRequest& request = dd->request;
    fcl::DistanceResult& result = dd->result;

    if(dd->done) {
        dist = result.min_distance;
        return true;
    }

    fcl::distance(o1, o2, request, result);
    dist = result.min_distance;

    if (dist <= 0) return true;
    return dd->done;
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
                                     std::map<std::string, double> fixed,
                                     std::vector<obstacle>& obstacles)
    : ompl::base::StateValidityChecker(si),
    robot_base(rb),
    fixed_joints(fixed),
    model(m),
    holding_object(false)
{
    world_ready = false;

    joint_names = model->get_joint_group(group);

    world = new fcl::DynamicAABBTreeCollisionManager();

    setup_obstacles(obstacles);

    world_ready = true;
}


collision_checker::collision_checker(const ompl::base::SpaceInformationPtr& si,
                                     std::shared_ptr<robot_model> m,
                                     transform3 rb,
                                     std::string group,
                                     std::map<std::string, double> fixed,
                                     std::vector<obstacle>& obstacles)
    : ompl::base::StateValidityChecker(si),
    robot_base(rb),
    fixed_joints(fixed),
    model(m),
    holding_object(false)
{
    world_ready = false;

    joint_names = model->get_joint_group(group);

    world = new fcl::DynamicAABBTreeCollisionManager();

    setup_obstacles(obstacles);

    world_ready = true;
}

collision_checker::collision_checker(ompl::base::SpaceInformation* si,
                                     std::shared_ptr<robot_model> m,
                                     transform3 rb,
                                     std::string group,
                                     std::map<std::string, double> fixed,
                                     std::vector<obstacle>& obstacles,
                                     obstacle held_object)
    : ompl::base::StateValidityChecker(si),
    robot_base(rb),
    fixed_joints(fixed),
    model(m),
    holding_object(true),
    held_object(held_object)
{
    world_ready = false;

    joint_names = model->get_joint_group(group);

    world = new fcl::DynamicAABBTreeCollisionManager();

    setup_obstacles(obstacles);

    if (held_object.geometry == BALL_OBSTACLE) {
        held_obj_geom =
            std::shared_ptr<fcl::CollisionGeometry>(new fcl::Sphere(held_object.ball_radius));
    } else if (held_object.geometry == BOX_OBSTACLE) {
        held_obj_geom =
            std::shared_ptr<fcl::CollisionGeometry>(
                new fcl::Box(held_object.box_dim[0],
                             held_object.box_dim[1],
                             held_object.box_dim[2]));
    } else if (held_object.geometry == CONVEX_OBSTACLE) {
        std::cout << "Warning: Collision detection for convex held object "
                  << held_object.name << " not supported" << std::endl;
        // XXX See note in setup_obstacles
    }

    world_ready = true;
}

collision_checker::collision_checker(const ompl::base::SpaceInformationPtr& si,
                                     std::shared_ptr<robot_model> m,
                                     transform3 rb,
                                     std::string group,
                                     std::map<std::string, double> fixed,
                                     std::vector<obstacle>& obstacles,
                                     obstacle held_object)
    : ompl::base::StateValidityChecker(si),
    robot_base(rb),
    fixed_joints(fixed),
    model(m),
    holding_object(true),
    held_object(held_object)
{
    world_ready = false;

    joint_names = model->get_joint_group(group);

    world = new fcl::DynamicAABBTreeCollisionManager();

    setup_obstacles(obstacles);

    if (held_object.geometry == BALL_OBSTACLE) {
        held_obj_geom =
            std::shared_ptr<fcl::CollisionGeometry>(new fcl::Sphere(held_object.ball_radius));
    } else if (held_object.geometry == BOX_OBSTACLE) {
        held_obj_geom =
            std::shared_ptr<fcl::CollisionGeometry>(
                new fcl::Box(held_object.box_dim[0],
                             held_object.box_dim[1],
                             held_object.box_dim[2]));
    } else if (held_object.geometry == CONVEX_OBSTACLE) {
        std::cout << "Warning: Collision detection for convex held object "
                  << held_object.name << " not supported" << std::endl;
        // XXX See note in setup_obstacles
    }

    world_ready = true;
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
        vec4 quat;
        i->transform.rotation(quat);
        fcl::Quaternion3f fcl_quat(quat[3], quat[0], quat[1], quat[2]);

        vec3 xyz;
        i->transform.position(xyz);
        fcl::Vec3f fcl_vec(xyz[0], xyz[1], xyz[2]);

        fcl::Transform3f fcl_xf(fcl_quat, fcl_vec);

        double PADDING = 0.01;
        if (i->geometry == BALL_OBSTACLE) {
            world_obj_geoms.push_back(std::shared_ptr<fcl::CollisionGeometry>(new fcl::Sphere(i->ball_radius + (PADDING/2.0))));
            geom_types.push_back(BALL_OBSTACLE);
        } else if (i->geometry == BOX_OBSTACLE) {
            world_obj_geoms.push_back(
                std::shared_ptr<fcl::CollisionGeometry>(
                    new fcl::Box(i->box_dim[0] + PADDING,
                                 i->box_dim[1] + PADDING,
                                 i->box_dim[2] + PADDING)));
            geom_types.push_back(BOX_OBSTACLE);
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
        world_obj_geoms.back()->setUserData(world_obj_datas.back());

        world->registerObject(world_objects.back());
    }
}

bool collision_checker::collide_internal(std::map<std::string, double> joint_state) const {
    fcl::BroadPhaseCollisionManager* robot = new fcl::DynamicAABBTreeCollisionManager();

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

    if (holding_object) {
        transform3 ee = model->end_effector_xform(joint_state);
        transform3 held = ee*held_object.transform;

        vec4 quat;
        held.rotation(quat);
        fcl::Quaternion3f fcl_quat(quat[3], quat[0], quat[1], quat[2]);
        vec3 pos;
        held.position(pos);
        fcl::Vec3f fcl_vec(pos[0], pos[1], pos[2]);
        fcl::Transform3f fcl_xf(fcl_quat, fcl_vec);

        obj_ptrs.push_back(new fcl::CollisionObject(held_obj_geom,
                                                    fcl_xf));
        obj_datas.push_back(new object_data());
        obj_datas.back()->name = held_object.name;
        obj_datas.back()->allowed_collisions = model->end_effector_links();
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

bool collision_checker::isValid(const ompl::base::State* state) const {
    // cast the abstract state type to the type we expect
    const ompl::base::RealVectorStateSpace::StateType* vecstate =
        state->as<ompl::base::RealVectorStateSpace::StateType>();

    // Robot parts are updated for every state
    std::map<std::string, double> joint_state;
    for (int i = 0; i < joint_names.size(); i++) {
        joint_state[joint_names[i]] = (*vecstate)[i];
    }
    std::map<std::string, double>::const_iterator j = fixed_joints.begin();
    for (; j != fixed_joints.end(); j++) {
        joint_state[j->first] = j->second;
    }

    return collide_internal(joint_state);
}

bool collision_checker::is_valid(std::vector<double> state) {
    std::map<std::string, double> joint_state;
    for (int i = 0; i < joint_names.size(); i++) {
        joint_state[joint_names[i]] = state[i];
    }
    std::map<std::string, double>::const_iterator j = fixed_joints.begin();
    for (; j != fixed_joints.end(); j++) {
        joint_state[j->first] = j->second;
    }

    return collide_internal(joint_state);
}

void collision_checker::print_scene(const ompl::base::State* state) {
    const ompl::base::RealVectorStateSpace::StateType* vecstate =
        state->as<ompl::base::RealVectorStateSpace::StateType>();
    std::map<std::string, double> joint_state;
    for (int i = 0; i < joint_names.size(); i++) {
        joint_state[joint_names[i]] = (*vecstate)[i];
    }

    print_scene(joint_state);
}

void collision_checker::print_scene(std::vector<double> state) {
    std::map<std::string, double> joint_state;
    for (int i = 0; i < joint_names.size(); i++) {
        joint_state[joint_names[i]] = state[i];
    }


    print_scene(joint_state);
}

void collision_checker::print_scene(std::map<std::string, double> joint_state) {
    std::map<std::string, double>::const_iterator j = fixed_joints.begin();
    for (; j != fixed_joints.end(); j++) {
        joint_state[j->first] = j->second;
    }

    // Asking for the transforms FOR THE MESH MODELS
    std::map<std::string, transform3> xforms = model->link_transforms(joint_state, false);

    // std::cout << "Joint state: " << std::endl;
    // for (std::map<std::string, double>::iterator js = joint_state.begin();
    //      js != joint_state.end(); js++) {
    //     std::cout << "   " << js->first << ": " << js->second << std::endl;
    // }

    if (!world_ready) {
        std::cout << "World objects empty!" << std::endl;
    } else {
        std::cout << "World objects manager has " << world->size()
                  << " objects:" << std::endl;
        std::vector<fcl::CollisionObject*>::iterator o = world_objects.begin();
        int index = 0;
        for (; o != world_objects.end(); o++) {
            object_data* od = static_cast<object_data*>((*o)->getUserData());
            std::cout << "    " << od->name << ": ";
            fcl::Vec3f p = (*o)->getTransform().getTranslation();
            fcl::Quaternion3f q = (*o)->getTransform().getQuatRotation();
            std::cout << "[" << p[0] << ", " << p[1] << ", " << p[2] << "]; ";
            std::cout << "[" << q[0] << ", " << q[1] << ", " << q[2]
                      << ", " << q[3] << "]; ";

            if (geom_types[index] == BOX_OBSTACLE) {
                std::shared_ptr<fcl::Box> b =
                    std::dynamic_pointer_cast<fcl::Box>(world_obj_geoms[index]);
                std::cout << "box [" << b->side[0] << ", " << b->side[1] << ", "
                          << b->side[2] << "]" << std::endl;
            } else if (geom_types[index] == BALL_OBSTACLE) {
                std::shared_ptr<fcl::Sphere> s =
                    std::dynamic_pointer_cast<fcl::Sphere>(world_obj_geoms[index]);
                std::cout << "sphere " << s->radius << std::endl;
            } else {
                std::cout << "???" << std::endl;
            }

            // Rotation matrix
            // fcl::Matrix3f mat = (*o)->getTransform().getRotation();
            // for (int r = 0; r < 3; r++) {
            //     std::cout << "        [";
            //     for (int c = 0; c < 3; c++) {
            //         std::cout << mat(r, c);
            //         if (c < 2) std::cout << ", ";
            //     }
            //     std::cout << "]" << std::endl;
            // }

            index++;
        }
    }

    // Create the robot collision manager as in a normal collision check
    fcl::BroadPhaseCollisionManager* robot = new fcl::DynamicAABBTreeCollisionManager();

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
        model->get_collision_model(t->first)->setUserData(obj_datas.back());
        robot->registerObject(obj_ptrs.back());
    }

    std::vector<fcl::CollisionObject*> robot_objects;
    robot->getObjects(robot_objects);
    std::cout << "Robot objects manager has " << robot->size()
              << " objects:" << std::endl;

    std::vector<fcl::CollisionObject*>::iterator r = robot_objects.begin();
    for (; r != robot_objects.end(); r++) {
        object_data* od = static_cast<object_data*>((*r)->getUserData());
        std::cout << "    " << od->name << ": ";
        fcl::Vec3f p = (*r)->getTransform().getTranslation();
        fcl::Quaternion3f q = (*r)->getTransform().getQuatRotation();
        std::cout << "[" << p[0] << ", " << p[1] << ", " << p[2] << "]; ";
        std::cout << "[" << q[0] << ", " << q[1] << ", " << q[2]
                  << ", " << q[3] << "]" << std::endl;

        // Rotation matrix
        // fcl::Matrix3f mat = (*r)->getTransform().getRotation();
        //     for (int w = 0; w < 3; w++) {
        //         std::cout << "        [";
        //         for (int c = 0; c < 3; c++) {
        //             std::cout << mat(w, c);
        //             if (c < 2) std::cout << ", ";
        //         }
        //         std::cout << "]" << std::endl;
        //     }
    }

    bool is_collision = false;

    // Self-collision
    collision_data cds;
    cds.request = fcl::CollisionRequest();
    cds.request.enable_contact = true;
    cds.request.enable_cost = false;
    cds.result = fcl::CollisionResult();
    robot->collide(&cds, collision_function);

    if (cds.result.isCollision()) {
        is_collision = true;
        std::cout << "|-----------------------------------------------|" << std::endl
                  << "|                                               |" << std::endl
                  << "|                SELF COLLISION                 |" << std::endl
                  << "|                                               |" << std::endl
                  << "|-----------------------------------------------|" << std::endl;

        for (int i = 0; i < cds.result.numContacts(); i++) {
            object_data* od1 =
                static_cast<object_data*>(cds.result.getContact(i).o1->getUserData());
            object_data* od2 =
                static_cast<object_data*>(cds.result.getContact(i).o2->getUserData());
            std::cout << "|                                               |" << std::endl
                      << "| Colliding objects: " << od1->name
                      << " and " << od2->name << std::endl
                      << "|                                               |" << std::endl
                      << "|-----------------------------------------------|" << std::endl;
        }
    }

    // World collision
    collision_data cdw;
    cdw.request = fcl::CollisionRequest();
    cdw.request.enable_contact = true;
    cdw.request.enable_cost = false;
    cdw.result = fcl::CollisionResult();
    robot->collide(world, &cdw, collision_function);

    if (cdw.result.isCollision()) {
        is_collision = true;
        std::cout << "|-----------------------------------------------|" << std::endl
                  << "|                                               |" << std::endl
                  << "|               WORLD COLLISION                 |" << std::endl
                  << "|                                               |" << std::endl
                  << "|-----------------------------------------------|" << std::endl;

        for (int i = 0; i < cdw.result.numContacts(); i++) {
            object_data* od1 =
                static_cast<object_data*>(cdw.result.getContact(i).o1->getUserData());
            object_data* od2 =
                static_cast<object_data*>(cdw.result.getContact(i).o2->getUserData());
            std::cout << "|                                               |" << std::endl
                      << "| Colliding objects: " << od1->name
                      << " and " << od2->name << std::endl
                      << "|                                               |" << std::endl
                      << "|-----------------------------------------------|" << std::endl;
        }
    }

    if (!is_collision) {
        std::cout << "|-----------------------------------------------|" << std::endl
                  << "|                                               |" << std::endl
                  << "|                 no collision                  |" << std::endl
                  << "|                                               |" << std::endl
                  << "|-----------------------------------------------|" << std::endl;
    }

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
}

double collision_checker::minimum_distance(std::vector<double> state) {
    fcl::BroadPhaseCollisionManager* robot = new fcl::DynamicAABBTreeCollisionManager();

    std::map<std::string, double> joint_state;
    for (int i = 0; i < joint_names.size(); i++) {
        joint_state[joint_names[i]] = state[i];
    }
    std::map<std::string, double>::const_iterator j = fixed_joints.begin();
    for (; j != fixed_joints.end(); j++) {
        joint_state[j->first] = j->second;
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

    // No self-distance needed
    // World distance
    distance_data dd;
    dd.request = fcl::DistanceRequest();
    dd.request.enable_nearest_points = false;
    dd.result = fcl::DistanceResult();
    robot->distance(world, &dd, distance_function);

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

    return dd.result.min_distance;
}

#endif
