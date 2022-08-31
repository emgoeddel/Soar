#ifdef ENABLE_ROS

#include "svs.h"
#include "symbol.h"
#include "command.h"
#include "command_table.h"
#include "scene.h"
#include "motor_state.h"
#include "motor/motor.h"

/*
 * straight_line_trajectory_command class
 *
 * Allows Soar to request a (short) straight-line movement rather than using
 * the full motion planning pipeline, specifically for grasp positioning. Assumes
 * that the gripper orientation should be maintained across the motion.
 *
 * Usage:
 *    ^target <vec3> - gripper target pose for the motion
 */

class straight_line_trajectory_command : public command {
public:
    straight_line_trajectory_command(svs_state* state, Symbol* root) : command(state, root),
                                                                       root(root),
                                                                       parsed(false) {
        si = state->get_svs()->get_soar_interface();
        ri = state->get_svs()->get_ros_interface();
        ms = state->get_motor_state();
        mtr = ms->get_motor();
    }

    std::string description() { return "straight-line-trajectory"; }
    int command_type() { return SVS_WRITE_COMMAND; }

    bool update_sub() {
        // When command is first put on the link, parse it
        if (!parsed) {
            parsed = true;
            // If parsed successfully, set the status to "running"
            if (parse()) {
                set_status("success");
            } else {
                // Error message already set in parse() method
                return false;
            }
            return true;
        }

        return true;
    }

private:
    bool parse() {
        std::cout << "Parsing a straight-line-trajectory command!!" << std::endl;

        if (!si->get_vec3(root, "target", gripper_target)) {
            set_status("no target found");
            return false;
        }

        if (!mtr->plan_straight_line(ms->get_joints(), t)) {
            set_status("planning_failure");
            return false;
        }

        return true;
    }

    soar_interface* si;
    ros_interface* ri;
    motor_state* ms;
    std::shared_ptr<motor> mtr;
    Symbol* root;

    bool parsed;
    vec3 gripper_target;
    trajectory t;
};

command* _make_straight_line_trajectory_command_(svs_state* state, Symbol* root)
{
    return new straight_line_trajectory_command(state, root);
}

command_table_entry* straight_line_trajectory_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "straight-line-trajectory";
    e->description = "Finds a straight-line trajectory to desired position";
    e->parameters["target"] = "Spatial target of the trajectories { ^x ^y ^z } ";
    e->create = &_make_straight_line_trajectory_command_;
    return e;
}

#endif
