#ifdef ENABLE_ROS

#include "svs.h"
#include "symbol.h"
#include "command.h"
#include "command_table.h"
#include "objective_table.h"
#include "scene.h"
#include "motor_state.h"

#include <fstream> // eval

/*
 * evaluate_trajectories_command class
 *
 * Defines the Soar interface for selecting <n> trajectories from a trajectory
 * set through a ^evaluate-trajectories command.
 *
 * Usage:
 *    ^set-id <id> - integer ID of trajectory set to eval
 *    ^type <t> - <select, value, rank>; type of output from objective
 *    ^objective <ob> - objective to base selection on
 *    ^number <n> - [Optional] number of trajectories to select *
 *    ^subset-type - [Optional] <strict, exact, loose> how to handle ties *
 *    ^direction <d> - [Optional] <max, min>; whether to maximize or minimize **
 *    ^update <true/false> - [Optional] whether to update output at each decision cyle ***
 *    ^previous-selection <name> - [Optional] run on subset from previous evaluation ****
 *
 * * For use with "select" only; subset-type specifies whether an <exact> subset of n is
 *   wanted, a <strict> subset that can be < n if a tie would take us over, or a <loose>
 *   subset that can be > n and will include all tied values
 * ** If not included, assumes minimize; ignored if using value
 * *** If not included, assumes false
 * **** Will only work if a previous objective with this name has made selections!
 */

class evaluate_trajectories_command : public command
{
public:
    evaluate_trajectories_command(svs_state* state, Symbol* root) : command(state, root),
                                                                root(root),
                                                                parsed(false),
                                                                update(false) {
        si = state->get_svs()->get_soar_interface();
        ms = state->get_motor_state();
    }

    std::string description() { return "evaluate-trajectories"; }
    int command_type() { return SVS_READ_COMMAND; }

    bool update_sub() {
        // When command is first put on the link, parse it
        if (!parsed) {
            parsed = true;
            if (parse()) {
                set_status("success");
                return true;
            } else return false; // status already set
        }

        if (update) {
            obj->get_latest_trajectories();

            if (!obj->evaluate()) {
                set_status("could not evaluate");
                return false;
            }

            if (obj->update_outputs()) {
                ms->update_objective_callback(traj_set_id, obj->get_name());
                set_status("success");
            }
        }

        return true;
    }

private:
    bool parse() {
        double set = -1;
        if (!si->get_const_attr(root, "set-id", set)) {
            set_status("no set-id found");
            return false;
        }
        traj_set_id = (int)set;
        if (!ms->has_set_id(traj_set_id)) {
            set_status("set-id not found in motor state");
            return false;
        }

        // EVAL ONLY
        if (ms->do_output()) {
            std::vector<std::string> names;
            names.push_back("planning-time");
            names.push_back("waypoints");
            names.push_back("execution-time");
            names.push_back("total-joint-movement");
            names.push_back("min-clearance");

            std::ofstream df;
            df.open("objectives.txt", std::ios::out | std::ios::app);
            if (!df.is_open()) std::cout << "ERROR writing to file!" << std::endl;
            df << ms->eval_objectives(traj_set_id, names) << std::endl;
            df.close();
        }
        // END EVAL

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

        bool use_previous_selection = false;
        std::string prev_selection_name;
        if (si->get_const_attr(root, "previous-selection", prev_selection_name)) {
            use_previous_selection = true;
            objective* prev_obj = ms->get_objective(traj_set_id, prev_selection_name);
            if (!prev_obj) {
                set_status("invalid previous selection");
                return false;
            }
            if (prev_obj->output_type() != SELECT) {
                set_status("previous must be select");
                return false;
            }
        }

        double num_traj = 1;
        si->get_const_attr(root, "number", num_traj);

        std::string subset_type = "";
        si->get_const_attr(root, "subset-type", subset_type);
        if (!(subset_type == "strict" || subset_type == "exact" ||
              subset_type == "loose" || subset_type == "")) {
            set_status("invalid subset-type given");
            return false;
        }

        std::string up;
        si->get_const_attr(root, "update", up);
        if (up == "true") update = true;

        std::string dir;
        bool max = false;
        si->get_const_attr(root, "direction", dir);
        if (dir == "max") {
             max = true;
        } else if (dir != "min" && dir != "") {
            set_status("invalid direction given");
            return false;
        }

        std::string obstacles;
        if (!si->get_const_attr(root, "obstacle", obstacles)) {
            std::stringstream ss;
            wme* obst_wme;
            if (si->find_child_wme(root, "obstacles", obst_wme)) {
                wme_vector obst_vec;
                if (si->get_child_wmes(si->get_wme_val(obst_wme), obst_vec)) {
                    wme_vector::iterator i = obst_vec.begin();
                    for (; i != obst_vec.end(); i++) {
                        std::string name;
                        if (!get_symbol_value(si->get_wme_val(*i), name)) {
                            set_status("invalid obstacles structure");
                            return false;
                        }
                        else ss << name << " ";  // XXX Major hack!
                    }
                }
            }
            obstacles = ss.str();
        }

        bool has_area_center = false;
        vec3 area_center;
        if (si->get_vec3(root, "area-center", area_center)) {
            has_area_center = true;
        }

        bool has_area_dimensions = false;
        vec3 area_dimensions;
        if (si->get_vec3(root, "area-dimensions", area_dimensions)) {
            has_area_dimensions = true;
        }

        bool has_area_radius = false;
        double area_radius;
        if (si->get_const_attr(root, "area-radius", area_radius)) {
            has_area_radius = true;
        }

        input = new objective_input();
        (*input)["output-type"] = new filter_val_c<std::string>(out_type);
        (*input)["name"] = new filter_val_c<std::string>(obj_name);
        (*input)["number"] = new filter_val_c<int>(num_traj);
        (*input)["maximize"] = new filter_val_c<bool>(max);
        (*input)["set-id"] = new filter_val_c<int>(traj_set_id);

        if (subset_type != "")
            (*input)["subset-type"] = new filter_val_c<std::string>(subset_type);

        if (use_previous_selection)
            (*input)["previous-selection"] =
                new filter_val_c<std::string>(prev_selection_name);

        if (obstacles != "")
            (*input)["obstacles"] = new filter_val_c<std::string>(obstacles);

        if (has_area_center)
            (*input)["area-center"] = new filter_val_c<vec3>(area_center);

        if (has_area_dimensions)
            (*input)["area-dimensions"] = new filter_val_c<vec3>(area_dimensions);

        if (has_area_radius)
            (*input)["area-radius"] = new filter_val_c<double>(area_radius);

        // Note obj is created here but deleted by the motor state later
        obj = get_objective_table().make_objective(obj_name,
                                                   root,
                                                   si,
                                                   ms,
                                                   input);

        obj->get_latest_trajectories();
        if (!obj->evaluate()) {
            set_status("could not evaluate");
            return false;
        }

        obj->update_outputs();
        ms->new_objective_callback(traj_set_id, obj);

        // EVAL ONLY
        if (ms->do_output()) {
            std::ofstream df2;
            df2.open("selections.txt", std::ios::out | std::ios::app);
            if (!df2.is_open()) std::cout << "ERROR writing to file!" << std::endl;
            df2 << traj_set_id << " "
                << ms->query_solve_time(traj_set_id) << " "
                << obj_name << " "
                << obj->get_selected() << " ";
            df2.close();
        }

        return true;
    }

    soar_interface* si;
    motor_state* ms;
    Symbol* root;

    objective* obj;
    objective_input* input; // owned by the objective though

    bool parsed;
    bool update;
    int traj_set_id;
};

command* _make_evaluate_trajectories_command_(svs_state* state, Symbol* root)
{
    return new evaluate_trajectories_command(state, root);
}

command_table_entry* evaluate_trajectories_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "evaluate-trajectories";
    e->description = "Evaluates trajectories in a set";
    e->parameters["set-id"] = "ID of trajectory set to evaluate";
    e->parameters["type"] = "Type of output from <value, rank, select>";
    e->parameters["objective"] = "Name of objective function to base selection on";
    e->parameters["number"] = "[Optional] Number of trajectories to return; default 1";
    e->parameters["update"] = "[Optional] Update output each decision cycle <true/false>; default false";
    e->create = &_make_evaluate_trajectories_command_;
    return e;
}

#endif
