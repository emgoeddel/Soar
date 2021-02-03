#ifdef ENABLE_ROS

#include "motor.h"

planning_problem::planning_problem(int qid, motor_query q, robot_model* m)
    : query_id(qid),
      query(q)
{
    joint_group = query.soar_query.joint_group;
    if (joint_group == "") joint_group = m->default_joint_group;

    // construct vector state space based on default joint group
    int dof = m->joint_groups[joint_group].size();
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(dof));

    // set the bounds for state space based on joint limits
    ompl::base::RealVectorBounds bounds(dof);
    std::vector<std::string>::iterator i = m->joint_groups[joint_group].begin();
    int b = 0;
    for (; i != m->joint_groups[joint_group].end(); i++)
    {
        std::string j = *i;
        bounds.setLow(b, m->all_joints[j].min_pos);
        bounds.setHigh(b, m->all_joints[j].max_pos);
        b++;
    }

    space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    ompl_ss = new ompl::geometric::SimpleSetup(space);
    cc = new collision_checker(ompl_ss->getSpaceInformation(), m, joint_group);
    ompl_ss->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(cc));

    std::cout << "Set up to plan for joint group " << joint_group.c_str()
              << ", DOF = " << dof << std::endl;
}

trajectory planning_problem::find_one() {
    std::cout << "Find a trajectory?" << std::endl;
    return trajectory();
}


motor::motor(std::string urdf) {
    model.init(urdf);
    std::cout << model.robot_info();
}

robot_model* motor::get_model_ptr() {
    if (model.name == "none") {
        std::cout << "Error: Robot model not initialized" << std::endl;
        return NULL;
    }
    return &model;
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
    trajectory t = ongoing.back().find_one();
    std::cout << "Found a trajectory of length " << t.length << std::endl;
    return true;
}

#endif
