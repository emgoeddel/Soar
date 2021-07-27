#ifdef ENABLE_ROS

#include "svs.h"
#include "symbol.h"
#include "command.h"
#include "command_table.h"
#include "scene.h"
#include "motor_state.h"

/*
 * select_trajectory_command class
 *
 * Defines the Soar interface for selecting <n> trajectories from a trajectory
 * set through a ^select-trajectory command.
 *
 * Usage:
 *    ^number <n> - [Optional] number of trajectories to select *
 *    ^objective <ob> - objective to base selection on
 *
 * * If not included, assumes select 1.
 */

class select_trajectory_command : public command
{
public:
    select_trajectory_command(svs_state* state, Symbol* root) : command(state, root),
                                                                root(root),
                                                                parsed(false) {
        si = state->get_svs()->get_soar_interface();
        ms = state->get_motor_state();
    }

    std::string description() { return "select-trajectory"; }
    int command_type() { return SVS_READ_COMMAND; }

    bool update_sub() {
        // When command is first put on the link, parse it
        if (!parsed) {
            parsed = true;
            if (parse()) {}
            else return false;
        }
        return true;
    }

private:
    bool parse() {
        std::cout << "Parsing a select-trajectory command!!" << std::endl;

        std::string obj_name;
        si->get_const_attr(root, "objective", obj_name);

        double num_traj = 0;
        si->get_const_attr(root, "number", num_traj);

        return true;
    }

    soar_interface* si;
    motor_state* ms;
    Symbol* root;

    bool parsed;
};

command* _make_select_trajectory_command_(svs_state* state, Symbol* root)
{
    return new select_trajectory_command(state, root);
}

command_table_entry* select_trajectory_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "select-trajectory";
    e->description = "Selects trajectories from a set";
    e->parameters["number"] = "[Optional] Number of trajectories to return; default 1";
    e->parameters["objective"] = "Name of objective function to base selection on";
    e->create = &_make_select_trajectory_command_;
    return e;
}

#endif
