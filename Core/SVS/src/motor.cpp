#ifdef ENABLE_ROS

#include "motor.h"


motor::motor(ros::NodeHandle& nh) : n(nh) {
    std::string rd = "";
    if (!n.getParam("/robot_description", rd)) {
        ROS_WARN("Can't find the robot_description parameter.");
    } else {
        model.init(rd);
    }

    //std::cout << model.robot_info();

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

// Calculate and return current link positions
std::map<std::string, transform3> motor::get_link_transforms() {
    std::lock_guard<std::mutex> guard(joints_mtx);
    return model.link_transforms(current_joints);
}

std::vector<std::string> motor::get_link_names() {
    std::vector<std::string> link_names;

    for (std::set<std::string>::iterator i = model.links_of_interest.begin();
         i != model.links_of_interest.end(); i++) {
        link_names.push_back(*i);
    }

    return link_names;
}

void motor::set_joints(std::map<std::string, double>& joints_in, bool verify) {
    std::lock_guard<std::mutex> guard(joints_mtx);

    for (std::map<std::string, double>::iterator i = joints_in.begin();
     i != joints_in.end(); i++) {
        if (verify && model.all_joints.count(i->first) == 0) {
            ROS_WARN("Joint name %s not present in motor model", i->first.c_str());
        }
        current_joints[i->first] = i->second;
    }

    if (!verify) return;

    for (std::map<std::string, joint_info>::iterator i = model.all_joints.begin();
         i != model.all_joints.end(); i++) {
        if (joints_in.count(i->first) == 0) {
            ROS_WARN("Model joint %s has no value in joint message", i->first.c_str());
        }
    }
}

std::map<std::string, double> motor::get_joints() {
    std::lock_guard<std::mutex> guard(joints_mtx);
    return current_joints;
}

#endif
