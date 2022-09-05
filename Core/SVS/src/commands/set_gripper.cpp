#ifdef ENABLE_ROS

#include "svs.h"
#include "symbol.h"
#include "command.h"
#include "command_table.h"
//#include "scene.h"
//#include "motor_state.h"

class set_gripper_command : public command {
public:
    set_gripper_command(svs_state* state, Symbol* root) : command(state, root),
                                                          root(root),
                                                          parsed(false) {
        si = state->get_svs()->get_soar_interface();
        ri = state->get_svs()->get_ros_interface();
    }

    std::string description() { return "set-gripper"; }
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

        bool finished = ri->gripper_done();
        if (finished && last_status == "running") {
            std::cout << "Gripper finished moving!" << std::endl;
            last_status = "finished";
            set_status(last_status);
            si->make_wme(root, "result", ri->gripper_result());
        }

        return true;
    }

private:
    bool parse() {
        std::cout << "Parsing a set-gripper command!!" << std::endl;

        if (!si->get_const_attr(root, "target", target_pose)) {
            set_status("no target position found");
            return false;
        }

        std::cout << "Setting gripper to " << target_pose << std::endl;
        ri->send_gripper_pos(target_pose);

        return true;
    }

    soar_interface* si;
    ros_interface* ri;
    Symbol* root;

    bool parsed;
    std::string last_status;
    double target_pose;
};

command* _make_set_gripper_command_(svs_state* state, Symbol* root)
{
    return new set_gripper_command(state, root);
}

command_table_entry* set_gripper_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "set-gripper";
    e->description = "Sets the gripper pose to a desired value";
    e->parameters["target"] = "Desired gripper pose [0.0 = closed, 0.1 = open]";
    e->create = &_make_set_gripper_command_;
    return e;
}

#endif
