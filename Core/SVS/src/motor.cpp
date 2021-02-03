#ifdef ENABLE_ROS

#include "motor.h"

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

planning_problem::planning_problem(int qid, motor_query q, robot_model* m)
    : query_id(qid),
      query(q),
      model(m)
{
    std::string joint_group = query.soar_query.joint_group;
    if (joint_group == "") joint_group = model->default_joint_group;
    joint_names = model->joint_groups[joint_group];

    // construct vector state space based on default joint group
    int dof = model->joint_groups[joint_group].size();
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(dof));

    // set the bounds for state space based on joint limits
    ompl::base::RealVectorBounds bounds(dof);
    std::vector<std::string>::iterator i = model->joint_groups[joint_group].begin();
    int b = 0;
    for (; i != model->joint_groups[joint_group].end(); i++)
    {
        std::string j = *i;
        bounds.setLow(b, model->all_joints[j].min_pos);
        bounds.setHigh(b, model->all_joints[j].max_pos);
        b++;
    }

    space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    ompl_ss = new ompl::geometric::SimpleSetup(space);

    robot = new fcl::DynamicAABBTreeCollisionManager();
    world = new fcl::DynamicAABBTreeCollisionManager();

    std::cout << "Set up to plan for joint group " << joint_group.c_str()
              << ", DOF = " << dof << std::endl;
}

bool planning_problem::state_valid(const ompl::base::State* state) {
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

motor::motor(std::string urdf) {
    model.init(urdf);

    std::cout << model.robot_info();
}

std::vector<std::string> motor::get_link_names() {
    std::vector<std::string> link_names;

    for (std::set<std::string>::iterator i = model.links_of_interest.begin();
         i != model.links_of_interest.end(); i++) {
        link_names.push_back(*i);
    }

    return link_names;
}

bool motor::new_planner_query(int id, motor_query q) {
    ongoing.push_back(planning_problem(id, q, &model));
    return true;
}

#endif
