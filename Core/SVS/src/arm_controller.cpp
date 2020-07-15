#ifdef ENABLE_ROS

#include "arm_controller.h"

arm_controller::arm_controller() : group("arm") {
    group.setMaxVelocityScalingFactor(0.4);
    group.setEndEffectorLink("gripper_link");
}

std::map<std::string, transform3> arm_controller::get_link_transforms() {
    std::map<std::string, transform3> xforms;

    robot_state::RobotStatePtr rs = group.getCurrentState();
    if (!rs) {
        std::cout << "Unable to get robot link transforms." << std::endl;
        return xforms;
    }

    std::vector<std::string> names = rs->getRobotModel()->getLinkModelNames();

    for (std::vector<std::string>::iterator i = names.begin();
         i != names.end(); i++) {
        std::cout << *i << std::endl;
    }
}

#endif
