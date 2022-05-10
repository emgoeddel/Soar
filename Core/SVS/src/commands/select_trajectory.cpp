#ifdef ENABLE_ROS

#include "svs.h"
#include "symbol.h"
#include "command.h"
#include "command_table.h"
#include "objective_table.h"
#include "scene.h"
#include "motor_state.h"

/*
 * select_trajectory_command class
 *
 * Defines the Soar interface for selecting <n> trajectories from a trajectory
 * set through a ^select-trajectory command.
 *
 * Usage:
 *    ^set-id <id> - integer ID of trajectory set to eval
 *    ^type <t> - <select, value, rank>; type of output from objective
 *    ^objective <ob> - objective to base selection on
 *    ^number <n> - [Optional] number of trajectories to select *
 *    ^update <true/false> - [Optional] whether to update output at each decision cyle **
 *
 * * If not included and using type select, assumes select 1; ignored if using other types
 * ** If not included, assumes false
 */

class select_trajectory_command : public command
{
public:
    select_trajectory_command(svs_state* state, Symbol* root) : command(state, root),
                                                                root(root),
                                                                parsed(false),
                                                                update(false) {
        si = state->get_svs()->get_soar_interface();
        ms = state->get_motor_state();
    }

    std::string description() { return "select-trajectory"; }
    int command_type() { return SVS_READ_COMMAND; }

    bool update_sub() {
        // When command is first put on the link, parse it
        if (!parsed) {
            parsed = true;
            if (parse()) {
                set_status("parsed");
                return true;
            } else return false;
        }

        if (update) {
            if (!obj->evaluate()) {
                set_status("could not evaluate");
                return false;
            }

            obj->update_outputs();
            // XXX ms->new_objective_callback(set_id, obj); Need update callback
        }

        return true;
    }

private:
    bool parse() {
        std::cout << "Parsing a select-trajectory command!!" << std::endl;

        double set = -1;
        if (!si->get_const_attr(root, "set-id", set)) {
            set_status("no set-id found");
            return false;
        }
        int set_id = (int)set;
        if (!ms->has_set_id(set_id)) {
            set_status("set-id not found in motor state");
            return false;
        }

        std::string out_type;
        if (!si->get_const_attr(root, "type", out_type)) {
            set_status("no type found");
            return false;
        }

        std::string obj_name;
        if (!si->get_const_attr(root, "objective", obj_name)) {
            set_status("no objective found");
            return false;
        }

        double num_traj = 1;
        si->get_const_attr(root, "number", num_traj);

        std::string up;
        si->get_const_attr(root, "update", up);
        if (up == "true") update = true;

        input = new objective_input();
        (*input)["output-type"] = new filter_val_c<std::string>(out_type);
        (*input)["name"] = new filter_val_c<std::string>(obj_name);
        (*input)["number"] = new filter_val_c<int>(num_traj);
        (*input)["set-id"] = new filter_val_c<int>(set_id);
        obj = get_objective_table().make_objective(obj_name,
                                                   root,
                                                   si,
                                                   ms,
                                                   input);

        if (!obj->evaluate()) {
            set_status("could not evaluate");
            return false;
        }

        obj->update_outputs(); // XXX Better way to do this?
        ms->new_objective_callback(set_id, obj);

        return true;
    }

    soar_interface* si;
    motor_state* ms;
    Symbol* root;

    objective* obj;
    objective_input* input; // owned by the objective though

    bool parsed;
    bool update;
};

command* _make_select_trajectory_command_(svs_state* state, Symbol* root)
{
    return new select_trajectory_command(state, root);
}

command_table_entry* select_trajectory_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "select-trajectory";
    e->description = "Evaluates trajectories in a set";
    e->parameters["set-id"] = "ID of trajectory set to evaluate";
    e->parameters["type"] = "Type of output from <value, rank, select>";
    e->parameters["objective"] = "Name of objective function to base selection on";
    e->parameters["number"] = "[Optional] Number of trajectories to return; default 1";
    e->parameters["update"] = "[Optional] Update output each decision cycle <true/false>; default false";
    e->create = &_make_select_trajectory_command_;
    return e;
}

#endif
