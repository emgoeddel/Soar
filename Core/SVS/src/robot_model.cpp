#ifdef ENABLE_ROS

#include "robot_model.h"

#include <random>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"

robot_model::robot_model() : name("none"),
                             ik_solver(NULL),
                             fk_solver(NULL) {}

robot_model::~robot_model() {
    if (ik_solver) delete ik_solver;
    if (fk_solver) delete fk_solver;
}

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

    // Adapted from fetch.srdf; commented out the ones that aren't links of interest
    //allowed["base_link"].insert("bellows_link");
    //allowed["base_link"].insert("bellows_link2");
    //allowed["base_link"].insert("estop_link");
    allowed["base_link"].insert("head_pan_link");
    allowed["base_link"].insert("head_tilt_link");
    //allowed["base_link"].insert("l_wheel_link");
    //allowed["base_link"].insert("laser_link");
    //allowed["base_link"].insert("r_wheel_link");
    allowed["base_link"].insert("shoulder_lift_link");
    allowed["base_link"].insert("shoulder_pan_link");
    //allowed["base_link"].insert("torso_fixed_link");
    allowed["base_link"].insert("torso_lift_link");
    allowed["base_link"].insert("upperarm_roll_link");
    // allowed["bellows_link"].insert("bellows_link2");
    // allowed["bellows_link"].insert("elbow_flex_link");
    // allowed["bellows_link"].insert("estop_link");
    // allowed["bellows_link"].insert("head_pan_link");
    // allowed["bellows_link"].insert("head_tilt_link");
    // allowed["bellows_link"].insert("l_wheel_link");
    // allowed["bellows_link"].insert("laser_link");
    // allowed["bellows_link"].insert("r_wheel_link");
    // allowed["bellows_link"].insert("shoulder_lift_link");
    // allowed["bellows_link"].insert("shoulder_pan_link");
    // allowed["bellows_link"].insert("torso_fixed_link");
    // allowed["bellows_link"].insert("torso_lift_link");
    // allowed["bellows_link"].insert("upperarm_roll_link");
    // allowed["bellows_link2"].insert("estop_link");
    // allowed["bellows_link2"].insert("head_pan_link");
    // allowed["bellows_link2"].insert("head_tilt_link");
    // allowed["bellows_link2"].insert("l_wheel_link");
    // allowed["bellows_link2"].insert("laser_link");
    // allowed["bellows_link2"].insert("r_wheel_link");
    // allowed["bellows_link2"].insert("shoulder_lift_link");
    // allowed["bellows_link2"].insert("shoulder_pan_link");
    // allowed["bellows_link2"].insert("torso_fixed_link");
    // allowed["bellows_link2"].insert("torso_lift_link");
    // allowed["bellows_link2"].insert("upperarm_roll_link");
    //allowed["elbow_flex_link"].insert("estop_link");
    allowed["elbow_flex_link"].insert("forearm_roll_link");
    allowed["elbow_flex_link"].insert("gripper_link");
    allowed["elbow_flex_link"].insert("l_gripper_finger_link");
    //allowed["elbow_flex_link"].insert("l_wheel_link");
    allowed["elbow_flex_link"].insert("r_gripper_finger_link");
    //allowed["elbow_flex_link"].insert("r_wheel_link");
    allowed["elbow_flex_link"].insert("shoulder_lift_link");
    allowed["elbow_flex_link"].insert("shoulder_pan_link");
    allowed["elbow_flex_link"].insert("upperarm_roll_link");
    allowed["elbow_flex_link"].insert("wrist_flex_link");
    allowed["elbow_flex_link"].insert("wrist_roll_link");
    // allowed["estop_link"].insert("forearm_roll_link");
    // allowed["estop_link"].insert("head_pan_link");
    // allowed["estop_link"].insert("head_tilt_link");
    // allowed["estop_link"].insert("l_wheel_link");
    // allowed["estop_link"].insert("laser_link");
    // allowed["estop_link"].insert("r_gripper_finger_link");
    // allowed["estop_link"].insert("r_wheel_link");
    // allowed["estop_link"].insert("shoulder_lift_link");
    // allowed["estop_link"].insert("shoulder_pan_link");
    // allowed["estop_link"].insert("torso_fixed_link");
    // allowed["estop_link"].insert("torso_lift_link");
    // allowed["estop_link"].insert("upperarm_roll_link");
    // allowed["estop_link"].insert("wrist_roll_link");
    allowed["forearm_roll_link"].insert("gripper_link");
    allowed["forearm_roll_link"].insert("l_gripper_finger_link");
    //allowed["forearm_roll_link"].insert("l_wheel_link");
    allowed["forearm_roll_link"].insert("r_gripper_finger_link");
    allowed["forearm_roll_link"].insert("shoulder_lift_link");
    allowed["forearm_roll_link"].insert("shoulder_pan_link");
    allowed["forearm_roll_link"].insert("upperarm_roll_link");
    allowed["forearm_roll_link"].insert("wrist_flex_link");
    allowed["forearm_roll_link"].insert("wrist_roll_link");
    allowed["gripper_link"].insert("l_gripper_finger_link");
    allowed["gripper_link"].insert("r_gripper_finger_link");
    allowed["gripper_link"].insert("upperarm_roll_link");
    allowed["gripper_link"].insert("wrist_flex_link");
    allowed["gripper_link"].insert("wrist_roll_link");
    allowed["head_pan_link"].insert("head_tilt_link");
    //allowed["head_pan_link"].insert("l_wheel_link");
    //allowed["head_pan_link"].insert("laser_link");
    //allowed["head_pan_link"].insert("r_wheel_link");
    allowed["head_pan_link"].insert("shoulder_lift_link");
    allowed["head_pan_link"].insert("shoulder_pan_link");
    //allowed["head_pan_link"].insert("torso_fixed_link");
    allowed["head_pan_link"].insert("torso_lift_link");
    //allowed["head_tilt_link"].insert("l_wheel_link");
    //allowed["head_tilt_link"].insert("laser_link");
    //allowed["head_tilt_link"].insert("r_wheel_link");
    allowed["head_tilt_link"].insert("shoulder_lift_link");
    allowed["head_tilt_link"].insert("shoulder_pan_link");
    //allowed["head_tilt_link"].insert("torso_fixed_link");
    allowed["head_tilt_link"].insert("torso_lift_link");
    //allowed["l_gripper_finger_link"].insert("l_wheel_link");
    allowed["l_gripper_finger_link"].insert("r_gripper_finger_link");
    allowed["l_gripper_finger_link"].insert("upperarm_roll_link");
    allowed["l_gripper_finger_link"].insert("wrist_flex_link");
    allowed["l_gripper_finger_link"].insert("wrist_roll_link");
    // allowed["l_wheel_link"].insert("laser_link");
    // allowed["l_wheel_link"].insert("r_gripper_finger_link");
    // allowed["l_wheel_link"].insert("r_wheel_link");
    // allowed["l_wheel_link"].insert("shoulder_lift_link");
    // allowed["l_wheel_link"].insert("shoulder_pan_link");
    // allowed["l_wheel_link"].insert("torso_fixed_link");
    // allowed["l_wheel_link"].insert("torso_lift_link");
    // allowed["l_wheel_link"].insert("upperarm_roll_link");
    // allowed["l_wheel_link"].insert("wrist_flex_link");
    // allowed["l_wheel_link"].insert("wrist_roll_link");
    // allowed["laser_link"].insert("r_gripper_finger_link");
    // allowed["laser_link"].insert("r_wheel_link");
    // allowed["laser_link"].insert("shoulder_lift_link");
    // allowed["laser_link"].insert("shoulder_pan_link");
    // allowed["laser_link"].insert("torso_fixed_link");
    // allowed["laser_link"].insert("torso_lift_link");
    // allowed["laser_link"].insert("upperarm_roll_link");
    allowed["r_gripper_finger_link"].insert("upperarm_roll_link");
    allowed["r_gripper_finger_link"].insert("wrist_flex_link");
    allowed["r_gripper_finger_link"].insert("wrist_roll_link");
    // allowed["r_wheel_link"].insert("shoulder_lift_link");
    // allowed["r_wheel_link"].insert("shoulder_pan_link");
    // allowed["r_wheel_link"].insert("torso_fixed_link");
    // allowed["r_wheel_link"].insert("torso_lift_link");
    // allowed["r_wheel_link"].insert("upperarm_roll_link");
    // allowed["r_wheel_link"].insert("wrist_roll_link");
    allowed["shoulder_lift_link"].insert("shoulder_pan_link");
    allowed["shoulder_lift_link"].insert("torso_fixed_link");
    allowed["shoulder_lift_link"].insert("upperarm_roll_link");
    allowed["shoulder_lift_link"].insert("wrist_flex_link");
    allowed["shoulder_lift_link"].insert("wrist_roll_link");
    //allowed["shoulder_pan_link"].insert("torso_fixed_link");
    allowed["shoulder_pan_link"].insert("torso_lift_link");
    allowed["shoulder_pan_link"].insert("wrist_flex_link");
    //allowed["torso_fixed_link"].insert("torso_lift_link");
    allowed["upperarm_roll_link"].insert("wrist_flex_link");
    allowed["upperarm_roll_link"].insert("wrist_roll_link");
    allowed["wrist_flex_link"].insert("wrist_roll_link");

    end_effector = "gripper_link";

    if (!kdl_parser::treeFromUrdfModel(urdf, kin_tree)){
        ROS_ERROR("Failed to construct KDL tree");
        return false;
    }
    kin_tree.getChain("torso_lift_link", end_effector, ik_chain);
    ik_solver = new KDL::ChainIkSolverPos_LMA(ik_chain);
    fk_solver = new KDL::ChainFkSolverPos_recursive(ik_chain);

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

std::vector<double>
robot_model::solve_ik(vec3 ee_pt) {
    std::vector<double> out;
    std::cout << "Trying to reach " << ee_pt[0] << ", " << ee_pt[1] << ", "
              << ee_pt[2] << std::endl;

    // XXX Baking in the assumption we are planning for the arm here, bad!
    if (ik_chain.getNrOfJoints() != joint_groups["arm"].size()) {
        std::cout << "Error: Kinematic chain does not represent Fetch arm!" << std::endl;
        return out;
    }

    KDL::Vector v(ee_pt[0], ee_pt[1], ee_pt[2]);
    KDL::Frame ee_desired(v);

    for (int tries = 0; tries < 20; tries++) {
        KDL::JntArray rnd_jnt(ik_chain.getNrOfJoints());
        std::vector<double> rnd = random_valid_pose("arm");
        for (int i = 0; i < rnd.size(); i++) {
            rnd_jnt.data[i] = rnd[i];
        }

        KDL::JntArray ik_sol(ik_chain.getNrOfJoints());
        int result = ik_solver->CartToJnt(rnd_jnt, ee_desired, ik_sol);
        // std::cout << "Result on it " << tries << " was ";
        // if (result == KDL::ChainIkSolverPos_LMA::E_NOERROR) {
        //     std::cout << "E_NOERROR";
        // } else if (result == KDL::ChainIkSolverPos_LMA::E_GRADIENT_JOINTS_TOO_SMALL) {
        //     std::cout << "E_GRADIENT_JOINTS_TOO_SMALL";
        // } else if (result == KDL::ChainIkSolverPos_LMA::E_INCREMENT_JOINTS_TOO_SMALL) {
        //     std::cout << "E_INCREMENT_JOINTS_TOO_SMALL";
        // } else if (result == KDL::ChainIkSolverPos_LMA::E_MAX_ITERATIONS_EXCEEDED) {
        //     std::cout << "E_MAX_ITERATIONS_EXCEEDED";
        // } else if (result == KDL::ChainIkSolverPos_LMA::E_SIZE_MISMATCH) {
        //     std::cout << "E_SIZE_MISMATCH";
        // } else {
        //     std::cout << "something else, " << result;
        // }
        // std::cout << std::endl;

        if (result == KDL::SolverI::E_NOERROR) {
            for (int i = 0; i < ik_sol.rows(); i++) {
                out.push_back(ik_sol(i));
            }
            std::cout << "IK solution found on try " << tries << std::endl;
            break;
        }
    }

    if (out.empty()) std::cout << "Error: No IK solution found" << std::endl;
    return out;
}

vec3 robot_model::end_effector_pos(std::map<std::string, double> joints) {
    KDL::JntArray jnt(ik_chain.getNrOfJoints());
    std::vector<std::string>::iterator i = joint_groups["arm"].begin();
    int ind = 0;
    for (; i != joint_groups["arm"].end(); i++) {
        jnt.data[ind] = joints[*i];
    }

    KDL::Frame f_out;
    fk_solver->JntToCart(jnt, f_out);

    // Just for a test...
    KDL::Chain chn;
    kin_tree.getChain(root_link, end_effector, chn);
    KDL::ChainFkSolverPos_recursive base_solver(chn);
    KDL::Frame tmp;
    KDL::JntArray fromBase(ik_chain.getNrOfJoints()+1);
    fromBase.data[0] = 0.2;
    std::vector<std::string>::iterator j = joint_groups["arm"].begin();
    ind = 1;
    for (; j != joint_groups["arm"].end(); j++) {
        fromBase.data[ind] = joints[*j];
    }
    int e = base_solver.JntToCart(fromBase, tmp);
    std::cout << "If solved from base: " << tmp.p.x() << ", " << tmp.p.y()
              << ", " << tmp.p.z() << " with err " << e << std::endl;
    //

    return vec3(f_out.p.x(), f_out.p.y(), f_out.p.z());
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

std::vector<double> robot_model::random_valid_pose(std::string group) {
    std::vector<double> out;
    std::random_device rd;
    std::default_random_engine dre(rd());
    std::uniform_real_distribution<double> dist(0, 1);

    for (std::vector<std::string>::iterator i = joint_groups[group].begin();
         i != joint_groups[group].end(); i++) {
        double r = dist(dre);
        if (all_joints[*i].type != CONTINUOUS) {
            out.push_back(all_joints[*i].min_pos + r * (all_joints[*i].max_pos -
                                                        all_joints[*i].min_pos));
        } else {
            out.push_back(-2*M_PI + r*4*M_PI);
        }
    }

    return out;
}

#endif