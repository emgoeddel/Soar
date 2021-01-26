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
    return get_link_transforms_at(current_joints);
}

// Calculate and return link positions for joint pose p
std::map<std::string, transform3>
motor::get_link_transforms_at(std::map<std::string, double> p) {
    std::map<std::string, transform3> xforms;

    // Put base link into the map immediately since it always starts
    // the kinematic chains as identity
    xforms[model.root_link] = transform3::identity();

    for (std::set<std::string>::iterator i = model.links_of_interest.begin();
         i != model.links_of_interest.end(); i++) {
        calculate_link_xform(*i, p, xforms);
    }

    return xforms;
}

// Add the xform for the requested link at pose p, plus any others along its
// kinematic chain, to the xform map
void motor::calculate_link_xform(std::string link_name,
                                 std::map<std::string, double> pose,
                                 std::map<std::string, transform3>& out) {
    // Already calculated as part of a previous link
    if (out.count(link_name) == 1) return;

    // Figure out all the links on the path to the link we're looking for
    // that have not already been calculated
    std::vector<std::string> chain_to_link;
    std::string l = link_name;
    while (out.count(l) == 0) {
        chain_to_link.push_back(l);
        // parent joint -> parent link
        std::string j = model.all_links[l].parent_joint;
        l = model.all_joints[j].parent_link;
    }
    // l must now have an entry in the xform map

    // Go through the chain starting with first existing xform
    transform3 cur_xform = out[l];
    for (std::vector<std::string>::reverse_iterator i = chain_to_link.rbegin();
         i != chain_to_link.rend(); i++) {
        std::string j = model.all_links[*i].parent_joint;
        transform3 j_x = compose_joint_xform(j, pose[j]);
        cur_xform = cur_xform*j_x;

        // Save xform for future if link is of interest before continuing
        if (model.links_of_interest.count(*i) ==  1) out[*i] = cur_xform;
    }
}

// Compose the innate and axis/angle or translation xform for the given joint
// at a particular position
transform3 motor::compose_joint_xform(std::string joint_name, double pos) {
    transform3 j_o = model.all_joints[joint_name].origin;
    vec3 j_axis = model.all_joints[joint_name].axis;

    if (model.all_joints[joint_name].type == REVOLUTE ||
        model.all_joints[joint_name].type == CONTINUOUS) {
        transform3 aa = transform3(j_axis, pos);
        return j_o*aa;
    } else if (model.all_joints[joint_name].type == PRISMATIC) {
        vec3 p(pos*j_axis[0], pos*j_axis[1], pos*j_axis[2]);
        transform3 t = transform3('p', p);
        return j_o*t;
    } else if (model.all_joints[joint_name].type == FIXED) {
        return j_o;
    } else {
        ROS_WARN("Unsupported joint %s needed for FK calculation", joint_name.c_str());
        return j_o;
    }
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

void motor::new_query(int id, query q) {
    std::cout << "Received query, what to do with it?" << std::endl;
    return;
}

#endif
