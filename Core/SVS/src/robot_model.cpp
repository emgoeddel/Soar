#ifdef ENABLE_ROS

#include "robot_model.h"

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
    joint_groups["default"] = std::vector<std::string>();
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

        // Convert to transform3 representation
        vec3 p(i->second->parent_to_joint_origin_transform.position.x,
               i->second->parent_to_joint_origin_transform.position.y,
               i->second->parent_to_joint_origin_transform.position.z);
        vec4 r(i->second->parent_to_joint_origin_transform.rotation.x,
               i->second->parent_to_joint_origin_transform.rotation.y,
               i->second->parent_to_joint_origin_transform.rotation.z,
               i->second->parent_to_joint_origin_transform.rotation.w);

        all_joints[n].origin = transform3(p, r);

        if (all_joints[n].type != FIXED &&
            all_joints[n].type != UNSUPPORTED) {

            all_joints[n].axis[0] = i->second->axis.x;
            all_joints[n].axis[1] = i->second->axis.y;
            all_joints[n].axis[2] = i->second->axis.z;

            all_joints[n].max_pos = i->second->limits->upper;
            all_joints[n].min_pos = i->second->limits->lower;
            all_joints[n].max_velocity = i->second->limits->velocity;
            all_joints[n].max_effort = i->second->limits->effort;
            joint_groups["default"].push_back(i->first);
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

        // Convert to transform3 representation
        vec3 c_p(j->second->collision->origin.position.x,
                 j->second->collision->origin.position.y,
                 j->second->collision->origin.position.z);
        vec4 c_r(j->second->collision->origin.rotation.x,
                 j->second->collision->origin.rotation.y,
                 j->second->collision->origin.rotation.z,
                 j->second->collision->origin.rotation.w);
        all_links[n].collision_origin = transform3(c_p, c_r);

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
    links_of_interest.insert("base_link");
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
    joint_groups["arm"] = std::vector<std::string>();
    joint_groups["arm"].push_back("shoulder_pan_joint");
    joint_groups["arm"].push_back("shoulder_lift_joint");
    joint_groups["arm"].push_back("upperarm_roll_joint");
    joint_groups["arm"].push_back("elbow_flex_joint");
    joint_groups["arm"].push_back("forearm_roll_joint");
    joint_groups["arm"].push_back("wrist_flex_joint");
    joint_groups["arm"].push_back("wrist_roll_joint");

    joint_groups["arm_w_torso"] = std::vector<std::string>();
    joint_groups["arm_w_torso"].push_back("torso_lift_joint");
    joint_groups["arm_w_torso"].push_back("shoulder_pan_joint");
    joint_groups["arm_w_torso"].push_back("shoulder_lift_joint");
    joint_groups["arm_w_torso"].push_back("upperarm_roll_joint");
    joint_groups["arm_w_torso"].push_back("elbow_flex_joint");
    joint_groups["arm_w_torso"].push_back("forearm_roll_joint");
    joint_groups["arm_w_torso"].push_back("wrist_flex_joint");
    joint_groups["arm_w_torso"].push_back("wrist_roll_joint");

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
    std::map<std::string, std::vector<std::string> >::iterator g = joint_groups.begin();
    for (; g != joint_groups.end(); g++) {
        info << "  [" << g->first << "] ";
        for (std::vector<std::string>::iterator i = g->second.begin();
             i != g->second.end(); i++) {
            info << *i << " ";
        }
        info << std::endl;
    }

    info << "===============================================================" << std::endl;
    return info.str();
}

#endif
