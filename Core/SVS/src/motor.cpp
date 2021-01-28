#ifdef ENABLE_ROS

#include "motor.h"


motor::motor(std::string urdf) {
    model.init(urdf);

    std::cout << model.robot_info();

    // construct vector state space based on default joint group
    int dof = model.joint_groups[model.default_joint_group].size();
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(dof));

    // set the bounds for state space based on joint limits
    ompl::base::RealVectorBounds bounds(dof);
    std::vector<std::string>::iterator i = model.joint_groups[model.default_joint_group].begin();
    int b = 0;
    for (; i != model.joint_groups[model.default_joint_group].end(); i++)
    {
        std::string j = *i;
        bounds.setLow(b, model.all_joints[j].min_pos);
        bounds.setHigh(b, model.all_joints[j].max_pos);
        b++;
    }

    space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    ompl_ss = new ompl::geometric::SimpleSetup(space);

    ROS_INFO("Set up to plan for joint group %s, DOF = %i",
             model.default_joint_group.c_str(), dof);
}

std::vector<std::string> motor::get_link_names() {
    std::vector<std::string> link_names;

    for (std::set<std::string>::iterator i = model.links_of_interest.begin();
         i != model.links_of_interest.end(); i++) {
        link_names.push_back(*i);
    }

    return link_names;
}

#endif
