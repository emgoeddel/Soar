#ifdef ENABLE_ROS

#include "arm_controller.h"

arm_controller::arm_controller() : group("arm") {
    group.setMaxVelocityScalingFactor(0.4);
    group.setEndEffectorLink("gripper_link");
}

#endif
