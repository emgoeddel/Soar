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

#include "mat.h"
#include "cliproxy.h"
#include "robot.h"

class svs;

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
    ros_interface(svs* sp);
    ~ros_interface();
    static void init_ros();
    void start_ros();
    void stop_ros();

    std::string get_image_source() { return image_source; }

    robot* get_robot_ptr() { return &fetch; }

private:
    static const double POS_THRESH;
    static const double ROT_THRESH;
    bool t_diff(vec3& p1, vec3& p2);
    bool t_diff(Eigen::Quaterniond& q1, Eigen::Quaterniond& q2);

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
    void update_robot(transform3 fetch_xform);
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
    ros::Subscriber pc_sub;
    std::string image_source;
    ros::AsyncSpinner* spinner;

    robot fetch;
    transform3 last_fetch;
    bool fetch_added;
    std::map<std::string, transform3> last_links;

    svs* svs_ptr;
    std::map<std::string, transform3> last_objs;
};

#endif
#endif
