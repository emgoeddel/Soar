#ifdef ENABLE_ROS

#include "motor_types.h"
#include "sgnode.h"

void from_sgnode(sgnode* node, obstacle& to) {
    to.name = node->get_id();

    std::string shape_info;
    node->get_shape_sgel(shape_info);

    if (shape_info == "") {
        std::cout << "Warning: Trying to turn group node " << to.name
                  << " into an obstacle!" << std::endl;
        to.geometry = NON_OBSTACLE;
    } else if (shape_info[0] == 'v') {
        to.geometry = CONVEX_OBSTACLE;
        convex_node* c = dynamic_cast<convex_node*>(node);
        for (std::vector<vec3>::const_iterator i = c->get_verts().begin();
             i != c->get_verts().end(); i++) {
            to.convex_pts.push_back(*i);
        }
    } else if (shape_info[0] == 'b') {
        to.geometry = BALL_OBSTACLE;
        ball_node* b = dynamic_cast<ball_node*>(node);
        to.ball_radius = b->get_radius();
    } else if (shape_info[0] == 'x') {
        to.geometry = BOX_OBSTACLE;
        box_node* x = dynamic_cast<box_node*>(node);
        to.box_dim = x->get_dimensions();
    } else {
        std::cout << "Warning: Unexpected geometry type for node " << to.name
                  << "!" << std::endl;
        to.geometry = NON_OBSTACLE;
    }

    node->get_trans(to.translation, to.rotation, to.scale);
}

void to_ros_msg(trajectory& from, trajectory_msgs::JointTrajectory& to) {
    to.header.frame_id = from.frame;
    to.joint_names = from.joints;
    to.points.clear();
    for (int i = 0; i < from.length; i++) {
        trajectory_msgs::JointTrajectoryPoint p;
        p.time_from_start = ros::Duration(from.times[i]);
        p.positions = from.waypoints[i];
        to.points.push_back(p);
    }
}

void from_ros_msg(trajectory_msgs::JointTrajectory& from, trajectory& to) {
    to.length = from.points.size();
    to.frame = from.header.frame_id;
    to.joints = from.joint_names;
    for (int i = 0; i < from.points.size(); i++) {
        trajectory_msgs::JointTrajectoryPoint p = from.points[i];
        to.waypoints.push_back(p.positions);
        to.times.push_back(p.time_from_start.toSec());
    }
}

#endif
