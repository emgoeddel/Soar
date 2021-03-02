#ifdef ENABLE_ROS

#include "svs.h"
#include "symbol.h"
#include "command.h"
#include "command_table.h"
#include "scene.h"
#include "motor_state.h"

class execute_trajectory_command : public command {
public:
    execute_trajectory_command(svs_state* state, Symbol* root) : command(state, root),
                                                                 root(root),
                                                                 parsed(false) {
        si = state->get_svs()->get_soar_interface();
        ms = state->get_motor_state();
    }

    std::string description() { return "execute-trajectory"; }
    int command_type() { return SVS_WRITE_COMMAND; }

    bool update_sub() {
        // When command is first put on the link, parse it
        if (!parsed) {
            parsed = true;
            // If parsed successfully, set the status to "running"
            if (parse()) {
                //ms->new_query(id, search_query);
                set_status("running");
            } else {
                // Error message already set in parse() method
                return false;
            }
        }
        return true;
    }

private:
    bool parse() {
        std::cout << "Parsing an execute-trajectory command!!" << std::endl;

        // Don't let the agent exec if this state's joints aren't the robot's
        // actual current joints
        if (ms->get_joints_type() != "current") {
            set_status("cannot execute from hypothetical joint state");
            return false;
        }

        wme* traj_wme;
        if (!si->find_child_wme(root, "trajectory", traj_wme)) {
            set_status("no trajectory selection found");
            return false;
        }
        // Compare to the trajectories in the set and figure out which one this
        // refers to
        trajectory t;
        if (!ms->match_trajectory(traj_wme, t)) {
            set_status("no corresponding trajectory in set");
            return false;
        }

        // Add check that trajectory's first state is pretty close to current
        // joint state.

        return true;
    }

    soar_interface* si;
    motor_state* ms;
    Symbol* root;

    bool parsed;
};

command* _make_execute_trajectory_command_(svs_state* state, Symbol* root)
{
    return new execute_trajectory_command(state, root);
}

command_table_entry* execute_trajectory_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "execute-trajectory";
    e->description = "Execute a selected trajectory";
    e->parameters["trajectory"] = "Trajectory to execute";
    e->create = &_make_execute_trajectory_command_;
    return e;
}

#endif
