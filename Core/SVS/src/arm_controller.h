#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#ifdef ENABLE_ROS

#include <moveit/move_group_interface/move_group_interface.h>

class arm_controller {
public:
    arm_controller();

private:
    moveit::planning_interface::MoveGroupInterface group;
};

#endif
#endif
