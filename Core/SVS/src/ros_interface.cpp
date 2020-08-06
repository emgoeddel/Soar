#ifdef ENABLE_ROS

#include "ros_interface.h"

#include <iostream>
#include <string>
#include <sstream>
#include <math.h>

#include "svs.h"

// Thresholds for determining when to update the scene graph
const double ros_interface::POS_THRESH = 0.001; // 1 mm
const double ros_interface::ROT_THRESH = 0.017; // approx 1 deg

const std::string ros_interface::IMAGE_NAME = "image";
const std::string ros_interface::OBJECTS_NAME = "objects";
const std::string ros_interface::ROBOT_NAME = "fetch";

ros_interface::ros_interface(svs* sp)
    : image_source("none"),
      fetch_added(false) {
    svs_ptr = sp;
    set_help("Control connections to ROS topics.");

    // We don't want all of the robot links in the SG (we don't need
    // to know where the e-stop is, for example). This holds the links
    // we actually need
    LINKS_OF_INTEREST.insert("elbow_flex_link");
    LINKS_OF_INTEREST.insert("forearm_roll_link");
    LINKS_OF_INTEREST.insert("gripper_link");
    LINKS_OF_INTEREST.insert("l_gripper_finger_link");
    LINKS_OF_INTEREST.insert("r_gripper_finger_link");
    LINKS_OF_INTEREST.insert("shoulder_lift_link");
    LINKS_OF_INTEREST.insert("shoulder_pan_link");
    LINKS_OF_INTEREST.insert("upperarm_roll_link");
    LINKS_OF_INTEREST.insert("wrist_flex_link");
    LINKS_OF_INTEREST.insert("wrist_roll_link");

    // Set up the maps needed to track which inputs are enabled/disabled
    // and change this via command line
    update_inputs[IMAGE_NAME] = false;
    update_inputs[OBJECTS_NAME] = false;
    update_inputs[ROBOT_NAME] = false;

    enable_fxns[IMAGE_NAME] = std::bind(&ros_interface::subscribe_image, this);
    enable_fxns[OBJECTS_NAME] = std::bind(&ros_interface::start_objects, this);
    enable_fxns[ROBOT_NAME] = std::bind(&ros_interface::start_robot, this);

    disable_fxns[IMAGE_NAME] = std::bind(&ros_interface::unsubscribe_image, this);
    disable_fxns[OBJECTS_NAME] = std::bind(&ros_interface::stop_objects, this);
    disable_fxns[ROBOT_NAME] = std::bind(&ros_interface::stop_robot, this);

    models_sub = n.subscribe("gazebo/model_states", 5, &ros_interface::objects_callback, this);
}

ros_interface::~ros_interface() {
    stop_ros();
}

// Static funtion to simply call ros::init, which MUST be called
// before creating the first NodeHandle. The NodeHandle is created
// in the class constructor, hence the need for this to be available
// without a class instance.
void ros_interface::init_ros() {
    int argc = 0;
    char* argv;

    ros::init(argc, &argv, "soar_svs");
}

void ros_interface::start_ros() {
    if (!spinner) {
        spinner = new ros::AsyncSpinner(4);
        spinner->start();
    }
}

void ros_interface::stop_ros() {
    if (spinner) spinner->stop();
    image_source = "none";
}

// Thresholded difference between two vectors. Allows us to not
// update the scene graph minor object movements that are below
// the threshold.
bool ros_interface::t_diff(vec3& p1, vec3& p2) {
    // Euclidean distance
    if (sqrt(pow(p1.x() - p2.x(), 2) +
             pow(p1.y() - p2.y(), 2) +
             pow(p1.z() - p2.z(), 2)) > POS_THRESH) return true;
    return false;
}

// Thresholded difference between two quaterions. Allows us to not
// update the scene graph minor object rotations that are below
// the threshold.
bool ros_interface::t_diff(Eigen::Quaterniond& q1, Eigen::Quaterniond& q2) {
    // Calculating the angle between to quaternions
    double a = 2 * acos(q1.dot(q2));
    if (a > ROT_THRESH) return true;
    return false;
}

// SGEL helper functions
std::string ros_interface::add_cmd(std::string name, std::string parent, vec3 p, vec3 r) {
    std::stringstream cmd;
    cmd << "add " << name << " " << parent;
    cmd << " p " << p.x() << " " << p.y() << " " << p.z();
    cmd << " r " << r.x() << " " << r.y() << " " << r.z();
    cmd << std::endl;
    return cmd.str();
}

std::string ros_interface::change_cmd(std::string name, vec3 p, vec3 r) {
    std::stringstream cmd;
    cmd << "change " << name;
    cmd << " p " << p.x() << " " << p.y() << " " << p.z();
    cmd << " r " << r.x() << " " << r.y() << " " << r.z();
    cmd << std::endl;
    return cmd.str();
}

std::string ros_interface::del_cmd(std::string name) {
    std::stringstream cmd;
    cmd << "delete " << name << std::endl;
    return cmd.str();
}

// Subscribes to the Fetch's point cloud topic
void ros_interface::subscribe_image() {
    pc_sub = n.subscribe("head_camera/depth_registered/points", 5, &ros_interface::pc_callback, this);
    image_source = "fetch";
    update_inputs[IMAGE_NAME] = true;
}

// Unsubscribes from the point cloud topic and updates Soar with
// an empty image
void ros_interface::unsubscribe_image() {
    pc_sub.shutdown();
    update_inputs[IMAGE_NAME] = false;
    image_source = "none";
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr empty(new pcl::PointCloud<pcl::PointXYZRGB>);
    pc_callback(empty);
}

// Subscribes to the models if needed (to get the location of the Fetch from gazebo)
// and turns on the robot update part of the callback
void ros_interface::start_robot() {
    update_inputs[ROBOT_NAME] = true;
}

// Stops the robot update part of the callback and removes the link objects from SG
void ros_interface::stop_robot() {
    update_inputs[ROBOT_NAME] = false;
}

// Subscribes to the models if needed and turns on the object update part
// of the callback
void ros_interface::start_objects() {
    update_inputs[OBJECTS_NAME] = true;
}

// Stops the object update part of the callback and delete objects from SG
void ros_interface::stop_objects() {
    update_inputs[OBJECTS_NAME] = false;
}

// Adds relevant commands to the input list in the main SVS class
// when a new world state is received from Gazebo
void ros_interface::objects_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    // First translate the message into a map of object names->xforms
    std::map<std::string, transform3> current_objs;
    // Save the Fetch transform separately
    transform3 fetch_loc;

    for (int i = 0; i <  msg->name.size(); i++) {
        geometry_msgs::Pose pose = msg->pose[i];
        std::string n = msg->name[i];

        vec3 p(pose.position.x,
               pose.position.y,
               pose.position.z);
        Eigen::Quaterniond q(pose.orientation.w,
                             pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z);
        // XXX: Is this right? Euler order in exsiting SVS?
        vec3 r = q.toRotationMatrix().eulerAngles(0, 1, 2);

        transform3 t(p, r, vec3(1, 1, 1));
        if (n == ROBOT_NAME) {
            fetch_loc = t;
        } else {
            current_objs.insert(std::pair<std::string, transform3>(n, t));
        }
    }

    update_objects(current_objs);
    update_robot(fetch_loc);
}

void ros_interface::update_robot(transform3 fetch_xform) {
    // Nothing to do if the robot input is off and it's not in the scene
    if (!update_inputs[ROBOT_NAME] && !fetch_added) return;

    // Build up a string of commands in the stringsream
    std::stringstream cmds;
    // But only bother sending the update to SVS if something changed
    bool robot_changed = false;


    vec3 fetch_pose;
    fetch_xform.position(fetch_pose);
    Eigen::Quaterniond rq;
    fetch_xform.rotation(rq);
    vec3 fetch_rot = rq.toRotationMatrix().eulerAngles(0, 1, 2);

    std::map<std::string, transform3> links = fetch.get_link_transforms();

    if (!update_inputs[ROBOT_NAME] && fetch_added) {
        // If we've turned off the robot updates and it's still in the scene, remove it
        robot_changed = true;
        cmds << del_cmd(ROBOT_NAME);
        fetch_added = false;
    } else if (!fetch_added) {
        // If this is the first update with the Fetch, add it to the scene
        robot_changed = true;
        // Add the Fetch base
        cmds << add_cmd(ROBOT_NAME, "world", fetch_pose, fetch_rot);

        // Add all the Fetch links as children of the above node
        for (std::map<std::string, transform3>::iterator i = links.begin();
             i != links.end(); i++) {
            std::string n = i->first;
            // Ignore the links we don't want
            if (LINKS_OF_INTEREST.count(n) == 0) continue;

            vec3 link_pose;
            i->second.position(link_pose);
            Eigen::Quaterniond lq;
            i->second.rotation(lq);
            vec3 link_rot = lq.toRotationMatrix().eulerAngles(0, 1, 2);

            cmds << add_cmd(n, ROBOT_NAME, link_pose, link_rot);
        }
        fetch_added = true;
    } else {
        // Check if the Fetch base has moved and update if so
        vec3 last_pose;
        last_fetch.position(last_pose);
        Eigen::Quaterniond pq;
        last_fetch.rotation(pq);
        vec3 last_rot = pq.toRotationMatrix().eulerAngles(0, 1, 2);

        if (t_diff(last_pose, fetch_pose) || t_diff(last_rot, fetch_rot)) {
            robot_changed = true;
            cmds << change_cmd(ROBOT_NAME, fetch_pose, fetch_rot);
        }

        // Check if the links have moved and update if so
        for (std::map<std::string, transform3>::iterator i = links.begin();
             i != links.end(); i++) {
            std::string n = i->first;
            // Ignore the links we don't want
            if (LINKS_OF_INTEREST.count(n) == 0) continue;

            vec3 link_pose;
            i->second.position(link_pose);
            Eigen::Quaterniond lq;
            i->second.rotation(lq);
            vec3 link_rot = lq.toRotationMatrix().eulerAngles(0, 1, 2);

            vec3 last_link_pose;
            last_links[n].position(last_link_pose);
            Eigen::Quaterniond llq;
            last_links[n].rotation(llq);
            vec3 last_link_rot = llq.toRotationMatrix().eulerAngles(0, 1, 2);

            if (t_diff(last_link_pose, link_pose) || t_diff(last_link_rot, link_rot)) {
                robot_changed = true;
                cmds << change_cmd(n, link_pose, link_rot);
            }
        }
    }

    last_fetch = fetch_xform;
    last_links = links;
    if (robot_changed) {
        // Send the compiled commands to the SVS input processor
        svs_ptr->add_input(cmds.str());
    }
}

// Create SVS commands for objects besides the Fetch
// XXX: Eventually want to use the map to directly update the scene graph
//      instead of going through SGEL. Will require threadsafe scene
//      graphs.
void ros_interface::update_objects(std::map<std::string, transform3> objs) {
    // If the input is off and we cleared the objects previously, nothing to do
    if (!update_inputs[OBJECTS_NAME] && last_objs.empty()) return;
    // IF the input is off and we HAVEN'T cleared the objects, delete them
    if (!update_inputs[OBJECTS_NAME]) objs.clear();

    // Build up a string of commands in the stringsream
    std::stringstream cmds;
    // But only bother sending the update to SVS if something changed
    bool objs_changed = false;

    // ADD commands for NEW objects that are present in the current msg
    // but are not present in SVS's scene graph
    for (std::map<std::string, transform3>::iterator i = objs.begin();
         i != objs.end(); i++) {
        if (last_objs.count(i->first) == 0) {
            objs_changed = true;
            std::string n = i->first;
            vec3 cur_pose;
            i->second.position(cur_pose);
            Eigen::Quaterniond rq;
            i->second.rotation(rq);
            vec3 cur_rot = rq.toRotationMatrix().eulerAngles(0, 1, 2);

            cmds << add_cmd(n, "world", cur_pose, cur_rot);
        }
    }

    for (std::map<std::string, transform3>::iterator i = last_objs.begin();
         i != last_objs.end(); i++) {
        // DELETE commands for objects that are NOT present in the msg but
        // are still present in SVS's scene graph
        if (objs.count(i->first) == 0) {
            objs_changed = true;

            cmds << del_cmd(i->first);
            continue;
        }

        // CHANGE commands for objects that are present in both the msg and
        // SVS scene graph but have changed position or rotation
        std::string n = i->first;
        vec3 last_pose;
        i->second.position(last_pose);
        vec3 cur_pose;
        objs[n].position(cur_pose);
        Eigen::Quaterniond last_rot;
        i->second.rotation(last_rot);
        Eigen::Quaterniond cur_rot;
        objs[n].rotation(cur_rot);
        vec3 cur_rot_rpy = cur_rot.toRotationMatrix().eulerAngles(0, 1, 2);

        // Check that at least one of the differences is above the thresholds
        if (t_diff(last_pose, cur_pose) || t_diff(last_rot, cur_rot)) {
            objs_changed = true;
            cmds << change_cmd(n, cur_pose, cur_rot_rpy);
        }
    }

    last_objs = objs;
    if (objs_changed) {
        // Send the compiled commands to the SVS input processor
        svs_ptr->add_input(cmds.str());
    }
}

// Updates the images in SVS states when a new point cloud is received
void ros_interface::pc_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg) {
    svs_ptr->image_callback(msg);
}

// Override of proxy_get_children from cliproxy; adds enable and disable
// methods to the svs ros command so that you can do svs ros.enable <INPUT>
// and svs ros.disable <INPUT>
void ros_interface::proxy_get_children(std::map<std::string, cliproxy*>& c) {
    c["enable"] = new memfunc_proxy<ros_interface>(this, &ros_interface::enable);
    c["enable"]->add_arg("INPUT", "Name of the input to enable.");
    c["disable"] = new memfunc_proxy<ros_interface>(this, &ros_interface::disable);
    c["disable"]->add_arg("INPUT", "Name of the input to disable.");
}

// Override of proxy_use_sub from cliproxy; this prints information about inputs
// and whether they're enabled/disabled when the user calls svs ros
void ros_interface::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os) {
    os << "=========== ROS INPUTS ===========" << std::endl;
    for (std::map<std::string, bool>::iterator i = update_inputs.begin();
         i != update_inputs.end(); i++) {
        os << i->first << ": ";
        if (i->second) os << "enabled";
        else os << "disabled";
        os << std::endl;
    }
    os << "==================================" << std::endl;
}

// The function called by the user command svs ros.enable; subscribes to the
// desired input (or all inputs), which will start their callbacks
void ros_interface::enable(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "Specify input to enable: all, ";
        for (std::map<std::string, bool>::iterator i = update_inputs.begin();
             i != update_inputs.end(); i++) {
            if (i != update_inputs.begin()) os << ", ";
            os << i->first;
        }
        os << std::endl;
        return;
    }

    if (args[0] == "all") {
        for (std::map<std::string, bool>::iterator i = update_inputs.begin();
             i != update_inputs.end(); i++) {
            if (!i->second) {
                enable_fxns[i->first]();
            }
        }
        os << "All ROS inputs enabled" << std::endl;
        return;
    }

    if (update_inputs.count(args[0]) == 0) {
        os << "Invalid input name provided" << std::endl;
        return;
    }

    if (!update_inputs[args[0]]) enable_fxns[args[0]]();
    os << args[0] << " ROS input enabled" << std::endl;
}

// The function called by the user command svs ros.disable; unsubscribes to the
// desired input (or all inputs), which will stop their callbacks
void ros_interface::disable(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "Specify input to disable: all, ";
        for (std::map<std::string, bool>::iterator i = update_inputs.begin();
             i != update_inputs.end(); i++) {
            if (i != update_inputs.begin()) os << ", ";
            os << i->first;
        }
        os << std::endl;
        return;
    }

    if (args[0] == "all") {
        for (std::map<std::string, bool>::iterator i = update_inputs.begin();
             i != update_inputs.end(); i++) {
            if (i->second) {
                disable_fxns[i->first]();
            }
        }
        os << "All ROS inputs disabled" << std::endl;
        return;
    }

    if (update_inputs.count(args[0]) == 0) {
        os << "Invalid input name provided" << std::endl;
        return;
    }

    if (update_inputs[args[0]]) disable_fxns[args[0]]();
    os << args[0] << " ROS input disabled" << std::endl;
}

#endif
