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
 * XXX Currently also handles execution of this trajectory!
 *
 * Usage:
 *    ^target <vec3> - gripper target for the motion
 *    ^type <pose relative> - [Optional] target is a 3D pose OR relative to current pose,
 *                                       pose is default
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

    std::string description() { return "straight-line-motion"; }
    int command_type() { return SVS_WRITE_COMMAND; }

    bool update_sub() {
        // When command is first put on the link, parse it
        if (!parsed) {
            parsed = true;
            // If parsed successfully, set the status to "running"
            if (parse()) {
                last_status = "running";
                set_status(last_status);
            } else {
                // Error message already set in parse() method
                return false;
            }
            return true;
        }

        bool finished = ri->execution_done();
        if (finished && last_status == "running") {
            std::cout << "Trajectory finished executing!" << std::endl;
            last_status = "finished";
            set_status(last_status);
            si->make_wme(root, "result", ri->execution_result());
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

        if (!si->get_const_attr(root, "type", type)) {
            type = "pose";
        }
        if (type != "pose" && type != "relative") {
            set_status("invalid target type");
            return false;
        }

        if (!mtr->plan_straight_line(ms->get_joints(), gripper_target, type, t)) {
            set_status("planning_failure");
            return false;
        }

        std::cout << "Executing straight-line trajectory with length "
                  << t.length << std::endl;
        ri->send_trajectory(t);
        return true;
    }

    soar_interface* si;
    ros_interface* ri;
    motor_state* ms;
    std::shared_ptr<motor> mtr;
    Symbol* root;
    // Symbol* result; XXX Report back to Soar

    bool parsed;
    vec3 gripper_target;
    std::string type;
    trajectory t;
    std::string last_status;
};

command* _make_straight_line_trajectory_command_(svs_state* state, Symbol* root)
{
    return new straight_line_trajectory_command(state, root);
}

command_table_entry* straight_line_trajectory_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "straight-line-motion";
    e->description = "Finds and executes a straight-line trajectory to desired position";
    e->parameters["target"] = "Spatial target of the trajectories { ^x ^y ^z } ";
    e->create = &_make_straight_line_trajectory_command_;
    return e;
}

#endif
