#ifdef ENABLE_ROS

#include "robot.h"

bool robot_model::init(std::string robot_desc) {
    urdf::Model urdf_model;
    if (!urdf_model.initString(robot_desc)) {
        ROS_WARN("Failed to parse URDF.");
        return false;
    }

    return true;
}

const std::string robot::ROBOT_NAME = "fetch";

robot::robot(ros::NodeHandle& nh) : n(nh),
                                    listener(tf_buffer),
                                    ompl_ss(std::make_shared<ompl::base::SE3StateSpace>()) {
    // We don't want all of the robot links in the SG (we don't need
    // to know where the e-stop is, for example). This holds the links
    // we actually need.
    // XXX: Obviously hard-coded to the fetch. Should eventually load
    //      from the robot model or something.
    LINKS_OF_INTEREST.insert("torso_lift_link");
    LINKS_OF_INTEREST.insert("head_pan_link");
    LINKS_OF_INTEREST.insert("head_tilt_link");
    LINKS_OF_INTEREST.insert("shoulder_pan_link");
    LINKS_OF_INTEREST.insert("shoulder_lift_link");
    LINKS_OF_INTEREST.insert("upperarm_roll_link");
    LINKS_OF_INTEREST.insert("elbow_flex_link");
    LINKS_OF_INTEREST.insert("forearm_roll_link");
    LINKS_OF_INTEREST.insert("wrist_flex_link");
    LINKS_OF_INTEREST.insert("wrist_roll_link");
    LINKS_OF_INTEREST.insert("gripper_link");
    LINKS_OF_INTEREST.insert("l_gripper_finger_link");
    LINKS_OF_INTEREST.insert("r_gripper_finger_link");

    std::string rd = "";
    if (!n.getParam("/robot_description", rd)) {
        ROS_WARN("Can't find the robot_description parameter.");
    } else {
        model.init(rd);
    }
}

// Query for current link positions through tf2
std::map<std::string, transform3> robot::get_link_transforms() {
    std::map<std::string, transform3> xforms;

    for (std::set<std::string>::iterator i = LINKS_OF_INTEREST.begin();
         i != LINKS_OF_INTEREST.end(); i++) {
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

    for (std::set<std::string>::iterator i = LINKS_OF_INTEREST.begin();
         i != LINKS_OF_INTEREST.end(); i++) {
        link_names.push_back(*i);
    }

    return link_names;
}

#endif
