#ifdef ENABLE_ROS

#include "robot.h"

robot::robot() : group("arm") {
    group.setMaxVelocityScalingFactor(0.4);
    group.setEndEffectorLink("gripper_link");
}

// Query for the current link positions through MoveIt!
std::map<std::string, transform3> robot::get_link_transforms() {
    std::map<std::string, transform3> xforms;

    robot_state::RobotStatePtr rs = group.getCurrentState();
    if (!rs) {
        std::cout << "Unable to get robot link transforms." << std::endl;
        return xforms;
    }

    std::vector<std::string> names = rs->getRobotModel()->getLinkModelNames();

    for (std::vector<std::string>::iterator i = names.begin();
         i != names.end(); i++) {
        std::string link_name = *i;
        xforms[link_name] = transform3(rs->getFrameTransform(link_name));
    }

    return xforms;
}

std::vector<std::string> robot::get_link_names() {
    robot_state::RobotStatePtr rs = group.getCurrentState();
    if (!rs) {
        std::cout << "Unable to get robot link names." << std::endl;
        std::vector<std::string> empty;
        return empty;
    }

    return rs->getRobotModel()->getLinkModelNames();
}

#endif
