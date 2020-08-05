#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#ifdef ENABLE_ROS

#include <map>
#include <moveit/move_group_interface/move_group_interface.h>

#include "mat.h"

class arm_controller {
public:
    arm_controller();
    std::map<std::string, transform3> get_link_transforms();
    std::vector<std::string> get_link_names();

private:
    moveit::planning_interface::MoveGroupInterface group;
};

#endif
#endif
