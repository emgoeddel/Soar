#ifdef ENABLE_ROS

#include "robot.h"

bool robot_model::init(std::string robot_desc) {
    urdf::Model urdf;
    if (!urdf.initString(robot_desc)) {
        ROS_WARN("Failed to parse URDF.");
        return false;
    }

    // Set the robot's name
    name = urdf.name_;

    // Get the root link for FK
    root_link = urdf.root_link_->name;

    default_joint_group = "default";

    // Get the joint information
    joint_groups["default"] = std::set<std::string>();
    std::map<std::string, urdf::JointSharedPtr>::iterator i = urdf.joints_.begin();
    for(; i != urdf.joints_.end(); i++) {
        std::string n = i->first;

        all_joints[n] = joint_info();
        all_joints[n].name = n;

        all_joints[n].parent_link = i->second->parent_link_name;
        all_joints[n].child_link = i->second->child_link_name;

        if (i->second->type == 1) {
            all_joints[n].type = REVOLUTE;
        } else if (i->second->type == 2) {
            all_joints[n].type = CONTINUOUS;
        }else if (i->second->type == 3) {
            all_joints[n].type = PRISMATIC;
        } else if (i->second->type == 6) {
            all_joints[n].type = FIXED;
        } else {
            ROS_WARN("%s has unsupported joint type.", n.c_str());
            all_joints[n].type = UNSUPPORTED;
        }

        // Convert to tf2 transform representation
        tf2::Vector3 v(i->second->parent_to_joint_origin_transform.position.x,
                       i->second->parent_to_joint_origin_transform.position.y,
                       i->second->parent_to_joint_origin_transform.position.z);
        all_joints[n].origin.setOrigin(v);
        double q_x, q_y, q_z, q_w;
        i->second->parent_to_joint_origin_transform.rotation.getQuaternion(q_x, q_y, q_z, q_w);
        tf2::Quaternion q(q_x, q_y, q_z, q_w);
        all_joints[n].origin.setRotation(q);

        if (all_joints[n].type != FIXED &&
            all_joints[n].type != UNSUPPORTED) {
            all_joints[n].max_pos = i->second->limits->upper;
            all_joints[n].min_pos = i->second->limits->lower;
            all_joints[n].max_velocity = i->second->limits->velocity;
            all_joints[n].max_effort = i->second->limits->effort;
            joint_groups["default"].insert(i->first);
        }
    }

    // Get the link information
    std::map<std::string, urdf::LinkSharedPtr>::iterator j = urdf.links_.begin();
    for (; j != urdf.links_.end(); j++) {
        std::string n = j->first;

        all_links[n] = link_info();
        all_links[n].name = n;

        if (n != root_link) {
            all_links[n].parent_joint = j->second->parent_joint->name;
        }
        all_links[n].child_joints = std::set<std::string>();
        std::vector<urdf::JointSharedPtr>::iterator k = j->second->child_joints.begin();
        for (; k != j->second->child_joints.end(); k++) {
            all_links[n].child_joints.insert((*k)->name);
        }

        if (!j->second->collision) {
            ROS_WARN("%s has no collision information.", n.c_str());
            continue;
        }

        // Convert to tf2 transform representation
        tf2::Vector3 v(j->second->collision->origin.position.x,
                       j->second->collision->origin.position.y,
                       j->second->collision->origin.position.z);
        all_links[n].collision_origin.setOrigin(v);
        double q_x, q_y, q_z, q_w;
        j->second->collision->origin.rotation.getQuaternion(q_x, q_y, q_z, q_w);
        tf2::Quaternion q(q_x, q_y, q_z, q_w);
        all_links[n].collision_origin.setRotation(q);

        // Mesh
        urdf::GeometrySharedPtr geom = j->second->collision->geometry;
        if (geom->type != 3) {
            ROS_WARN("%s does not have a mesh geometry.", n.c_str());
            continue;
        }
        std::shared_ptr<urdf::Mesh> m = std::static_pointer_cast<urdf::Mesh>(geom);
        all_links[n].mesh_file = m->filename;
    }

    // FETCH-SPECIFIC SETUP
    // XXX Move into config files eventually

    // Fetch-specific link information
    // We don't want all of the robot links in the SG (we don't need
    // to know where the e-stop is, for example). This holds the links
    // we actually need.
    links_of_interest.insert("torso_lift_link");
    links_of_interest.insert("head_pan_link");
    links_of_interest.insert("head_tilt_link");
    links_of_interest.insert("shoulder_pan_link");
    links_of_interest.insert("shoulder_lift_link");
    links_of_interest.insert("upperarm_roll_link");
    links_of_interest.insert("elbow_flex_link");
    links_of_interest.insert("forearm_roll_link");
    links_of_interest.insert("wrist_flex_link");
    links_of_interest.insert("wrist_roll_link");
    links_of_interest.insert("gripper_link");
    links_of_interest.insert("l_gripper_finger_link");
    links_of_interest.insert("r_gripper_finger_link");

    // Fetch-specific joint information
    // Groups of joints that we would actually want to plan for
    joint_groups["arm"] = std::set<std::string>();
    joint_groups["arm"].insert("shoulder_pan_joint");
    joint_groups["arm"].insert("shoulder_lift_joint");
    joint_groups["arm"].insert("upperarm_roll_joint");
    joint_groups["arm"].insert("elbow_flex_joint");
    joint_groups["arm"].insert("forearm_roll_joint");
    joint_groups["arm"].insert("wrist_flex_joint");
    joint_groups["arm"].insert("wrist_roll_joint");

    joint_groups["arm_w_torso"] = std::set<std::string>();
    joint_groups["arm_w_torso"].insert("torso_lift_joint");
    joint_groups["arm_w_torso"].insert("shoulder_pan_joint");
    joint_groups["arm_w_torso"].insert("shoulder_lift_joint");
    joint_groups["arm_w_torso"].insert("upperarm_roll_joint");
    joint_groups["arm_w_torso"].insert("elbow_flex_joint");
    joint_groups["arm_w_torso"].insert("forearm_roll_joint");
    joint_groups["arm_w_torso"].insert("wrist_flex_joint");
    joint_groups["arm_w_torso"].insert("wrist_roll_joint");

    default_joint_group = "arm";

    return true;
}

std::string robot_model::robot_info() {
    std::stringstream info;
    info << "========================= ROBOT MODEL =========================" << std::endl;
    info << "| NAME | " << name << std::endl;
    info << "---------------------------------------------------------------" << std::endl;
    info << "| LINKS | ";
    std::map<std::string, link_info>::iterator l = all_links.begin();
    for (; l != all_links.end(); l++) {
        info << l->first << " ";
    }
    info << std::endl;
    info << "---------------------------------------------------------------" << std::endl;
    info << "| SELECTED LINKS | ";
    for (std::set<std::string>::iterator i = links_of_interest.begin();
         i != links_of_interest.end(); i++) {
        info << *i << " ";
    }
    info << std::endl;
    info << "---------------------------------------------------------------" << std::endl;
    info << "| JOINTS | ";
    std::map<std::string, joint_info>::iterator j = all_joints.begin();
    for (; j != all_joints.end(); j++) {
        info << j->first << " ";
    }
    info << std::endl;
    info << "---------------------------------------------------------------" << std::endl;
    info << "| JOINT GROUPS |" << std::endl;;
    std::map<std::string, std::set<std::string> >::iterator g = joint_groups.begin();
    for (; g != joint_groups.end(); g++) {
        info << "  [" << g->first << "] ";
        for (std::set<std::string>::iterator i = g->second.begin();
             i != g->second.end(); i++) {
            info << *i << " ";
        }
        info << std::endl;
    }

    info << "===============================================================" << std::endl;
    return info.str();
}

robot::robot(ros::NodeHandle& nh) : n(nh),
                                    listener(tf_buffer) {
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
    std::set<std::string>::iterator i = model.joint_groups[model.default_joint_group].begin();
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

// Query for current link positions through tf2
std::map<std::string, transform3> robot::get_link_transforms() {
    std::map<std::string, transform3> xforms;

    for (std::set<std::string>::iterator i = model.links_of_interest.begin();
         i != model.links_of_interest.end(); i++) {
        std::string cur_link = *i;
        geometry_msgs::TransformStamped xf;
        try {
            xf = tf_buffer.lookupTransform("base_link", cur_link, ros::Time(0));
            xforms[cur_link] = transform3(tf2::transformToEigen(xf));
        } catch (tf2::TransformException &e) {
            ROS_WARN("%s", e.what());
        }
    }

    return xforms;
}

std::vector<std::string> robot::get_link_names() {
    std::vector<std::string> link_names;

    for (std::set<std::string>::iterator i = model.links_of_interest.begin();
         i != model.links_of_interest.end(); i++) {
        link_names.push_back(*i);
    }

    return link_names;
}

#endif
