#ifdef ENABLE_ROS

#include "ros_interface.h"

#include <iostream>
#include <string>
#include <sstream>
#include <math.h>

#include <urdf/model.h>

#include "svs.h"
#include "model_database.h"

const std::string ros_interface::IMAGE_NAME = "image";
const std::string ros_interface::OBJECTS_NAME = "objects";

ros_interface::ros_interface(svs* sp, std::shared_ptr<model_database> md)
    : image_source("none"),
      spinner(NULL),
      axn_client(n, "/arm_controller/follow_joint_trajectory", true),
      model_db(md)
{
    svs_ptr = sp;
    set_help("Control connections to ROS topics.");

    // Pull the robot description from the params server and save
    if (!n.getParam("/robot_description", robot_desc)) {
        ROS_WARN("Can't find the robot_description parameter.");
    }
    urdf::Model urdf;
    if (!urdf.initString(robot_desc)) {
        ROS_WARN("Failed to parse URDF.");
    }
    robot_name = urdf.name_;

    // Set up the maps needed to track which inputs are enabled/disabled
    // and change this via command line
    update_inputs[IMAGE_NAME] = false;
    update_inputs[OBJECTS_NAME] = false;
    update_inputs[robot_name] = false;

    enable_fxns[IMAGE_NAME] = std::bind(&ros_interface::subscribe_image, this);
    enable_fxns[OBJECTS_NAME] = std::bind(&ros_interface::start_objects, this);
    enable_fxns[robot_name] = std::bind(&ros_interface::start_robot, this);

    disable_fxns[IMAGE_NAME] = std::bind(&ros_interface::unsubscribe_image, this);
    disable_fxns[OBJECTS_NAME] = std::bind(&ros_interface::stop_objects, this);
    disable_fxns[robot_name] = std::bind(&ros_interface::stop_robot, this);

    // Stay subscribed to the gazebo models for the entire runtime
    models_sub = n.subscribe("gazebo/model_states", 5, &ros_interface::models_callback, this);
    // Same for the joint states
    joints_sub = n.subscribe("/joint_states", 5, &ros_interface::joints_callback, this);

    // Make sure the action client can connect to the server
    axn_client.waitForServer();
    ROS_INFO("Connected to joint trajectory action server.");
}

ros_interface::~ros_interface() {
    stop_ros();
    if(spinner) delete spinner;
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

void ros_interface::send_trajectory(trajectory& t) {
    control_msgs::FollowJointTrajectoryGoal goal_msg;
    to_ros_msg(t, goal_msg.trajectory);
    std::cout << "Message has length " << goal_msg.trajectory.points.size()
              << std::endl;
    axn_client.sendGoal(goal_msg);
}

bool ros_interface::execution_done() {
    return axn_client.getState().isDone();
}

std::string ros_interface::execution_result() {
    return axn_client.getState().toString();
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

// Turns on the robot update part of the model callback
void ros_interface::start_robot() {
    update_inputs[robot_name] = true;
}

// Turns off the robot update part of the model callback
void ros_interface::stop_robot() {
    update_inputs[robot_name] = false;
    std::map<std::string, double> empty;
    svs_ptr->add_joint_input(empty);
}

// Turns on the non-robot object part of the model callback
void ros_interface::start_objects() {
    update_inputs[OBJECTS_NAME] = true;
}

// Turns off the non-robot object part of the model callback
void ros_interface::stop_objects() {
    update_inputs[OBJECTS_NAME] = false;
}

// Callback for the incoming gazebo models, handles calling the robot and
// object update functions with the right input if they are turned on
void ros_interface::models_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
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
        vec3 r = q.toRotationMatrix().eulerAngles(0, 1, 2);

        transform3 t(p, r, vec3(1, 1, 1));
        if (n == robot_name) {
            fetch_loc = t;
        } else {
            current_objs.insert(std::pair<std::string, transform3>(n, t));
        }
    }

    update_objects(current_objs);
    if (update_inputs[robot_name]) svs_ptr->add_loc_input(fetch_loc);
}

void ros_interface::joints_callback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (!update_inputs[robot_name]) return;

    std::map<std::string, double> joints_in;
    for (int i = 0; i < msg->name.size(); i++) {
        joints_in[msg->name[i]] = msg->position[i];
    }
    svs_ptr->add_joint_input(joints_in);
}

// Updates the positions of objects besides the fetch in the scene graph
// via SGEL commands. This function could be called by perception
// instead of getting these locations from the simulator in a more complete
// system in the future.
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
        if (transform3::t_diff(i->second, objs[n])) {
            objs_changed = true;
            cmds << change_cmd(n, cur_pose, cur_rot_rpy);
        }
    }

    last_objs = objs;
    if (objs_changed) {
        // Send the compiled commands to the SVS input processor
        svs_ptr->add_sgel_input(cmds.str());
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
