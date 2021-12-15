#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#ifdef ENABLE_ROS

#include <functional>
#include <map>
#include <set>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/JointState.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "mat.h"
#include "cliproxy.h"
#include "motor_types.h"

class svs;
class model_database;

/*
 * ros_interface class
 *
 * Provides the necessary boilerplate to make SVS into a ROS
 * node, such as the NodeHandle, init functionality, and subscribers.
 * Includes callback functions that take ROS input and put point
 * cloud data into the image holders and update the scene graph from
 * Gazebo objects.
 *
 * CLI USAGE:
 *
 * svs ros - Prints all inputs and whether they're enabled or disabled
 * svs ros.enable <NAME> - Enables input with name <NAME>
 * svs ros.disable <NAME> - Disables input with name <NAME>
 *
 * <NAME> = "all" (enables/disables all inputs) OR one of inputs listed
 *          by the svs ros command
 */

class ros_interface : public cliproxy {
public:
    ros_interface(svs* sp, std::shared_ptr<model_database> md);
    ~ros_interface();
    static void init_ros();
    void start_ros();
    void stop_ros();

    void send_trajectory(trajectory& t);
    bool execution_done();
    std::string execution_result();

    std::string get_image_source() { return image_source; }
    std::string get_robot_desc() { return robot_desc; }

private:
    static std::string add_cmd(std::string name, std::string parent, vec3 p, vec3 r);
    static std::string change_cmd(std::string name, vec3 p, vec3 r);
    static std::string del_cmd(std::string name);

    static const std::string IMAGE_NAME;
    static const std::string OBJECTS_NAME;

    void subscribe_image();
    void unsubscribe_image();
    void start_robot();
    void stop_robot();
    void start_objects();
    void stop_objects();
    void models_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void joints_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void update_objects(std::map<std::string, transform3> objs);
    void pc_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg);

    void proxy_get_children(std::map<std::string, cliproxy*>& c);
    void proxy_use_sub(const std::vector<std::string>& args, std::ostream& os);
    void enable(const std::vector<std::string>& args, std::ostream& os);
    void disable(const std::vector<std::string>& args, std::ostream& os);

    ros::NodeHandle n;
    std::map<std::string, bool> update_inputs;
    std::map<std::string, std::function<void()> > enable_fxns;
    std::map<std::string, std::function<void()> > disable_fxns;
    ros::Subscriber models_sub;
    ros::Subscriber joints_sub;
    ros::Subscriber pc_sub;
    std::string image_source;
    ros::AsyncSpinner* spinner;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> axn_client;

    std::string robot_desc;
    std::string robot_name;

    svs* svs_ptr;
    std::shared_ptr<model_database> model_db;
    std::map<std::string, transform3> last_objs;
};

#endif
#endif
