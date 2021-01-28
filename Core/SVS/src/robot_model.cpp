#ifdef ENABLE_ROS

#include "robot_model.h"

#include <urdf/model.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"


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
        shapes::Mesh* mesh_obj = shapes::createMeshFromResource(m->filename);

        // Save vertices for SVS and collision object for motor
        std::vector<fcl::Vec3f> points(mesh_obj->vertex_count);
        for (int v = 0; v < mesh_obj->vertex_count; v++) {
            all_links[n].vertices.push_back(vec3(mesh_obj->vertices[3*v],
                                                 mesh_obj->vertices[3*v + 1],
                                                 mesh_obj->vertices[3*v + 2]));
            points[v] = fcl::Vec3f(mesh_obj->vertices[3*v],
                                   mesh_obj->vertices[3*v + 1],
                                   mesh_obj->vertices[3*v + 2]);
        }
        std::vector<fcl::Triangle> triangles(mesh_obj->triangle_count);
        for (int t = 0; t < mesh_obj->triangle_count; t++) {
            triangles[t] = fcl::Triangle(mesh_obj->triangles[3*t],
                                         mesh_obj->triangles[3*t + 1],
                                         mesh_obj->triangles[3*t + 2]);
        }
        all_links[n].collision_model.beginModel();
        all_links[n].collision_model.addSubModel(points, triangles);
        all_links[n].collision_model.endModel();
        delete mesh_obj;
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

// Calculate and return link positions for joint pose p
std::map<std::string, transform3>
robot_model::link_transforms(std::map<std::string, double> p) {
    std::map<std::string, transform3> xforms;

    // Put base link into the map immediately since it always starts
    // the kinematic chains as identity
    xforms[root_link] = transform3::identity();

    for (std::set<std::string>::iterator i = links_of_interest.begin();
         i != links_of_interest.end(); i++) {
        calculate_link_xform(*i, p, xforms);
    }

    return xforms;
}

// Add the xform for the requested link at pose p, plus any others along its
// kinematic chain, to the xform map
void robot_model::calculate_link_xform(std::string link_name,
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
        std::string j = all_links[l].parent_joint;
        l = all_joints[j].parent_link;
    }
    // l must now have an entry in the xform map

    // Go through the chain starting with first existing xform
    transform3 cur_xform = out[l];
    for (std::vector<std::string>::reverse_iterator i = chain_to_link.rbegin();
         i != chain_to_link.rend(); i++) {
        std::string j = all_links[*i].parent_joint;
        transform3 j_x = compose_joint_xform(j, pose[j]);
        cur_xform = cur_xform*j_x;

        // Save xform for future if link is of interest before continuing
        if (links_of_interest.count(*i) ==  1) out[*i] = cur_xform;
    }
}

// Compose the innate and axis/angle or translation xform for the given joint
// at a particular position
transform3 robot_model::compose_joint_xform(std::string joint_name, double pos) {
    transform3 j_o = all_joints[joint_name].origin;
    vec3 j_axis = all_joints[joint_name].axis;

    if (all_joints[joint_name].type == REVOLUTE ||
        all_joints[joint_name].type == CONTINUOUS) {
        transform3 aa = transform3(j_axis, pos);
        return j_o*aa;
    } else if (all_joints[joint_name].type == PRISMATIC) {
        vec3 p(pos*j_axis[0], pos*j_axis[1], pos*j_axis[2]);
        transform3 t = transform3('p', p);
        return j_o*t;
    } else if (all_joints[joint_name].type == FIXED) {
        return j_o;
    } else {
        ROS_WARN("Unsupported joint %s needed for FK calculation", joint_name.c_str());
        return j_o;
    }
}

#endif
