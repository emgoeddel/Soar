#ifdef ENABLE_ROS

#include "svs.h"
#include "symbol.h"
#include "command.h"
#include "command_table.h"
#include "scene.h"
#include "motor_state.h"

/*
 * find_trajectories_command class
 *
 * Defines the Soar interface for requesting and controlling the
 * creation of a MAST trajectory set through a ^find-trajectories command.
 *
 * Usage:
 *    ^target <t> - gripper target area for the motion
 *        ^center <vec3> - center of the target area
 *        ^frame <world self> - coord frame in which center is expressed
 *        ^size <vec3> OR <double> - if vec3, half box dimensions; if double, radius
 *        ^orientation <vec3> - [Optional] gripper orientation rpy
 *        ^orientation-flex <double> - [Optional] accpetable variation from target orientation
 *    ^min-number - [Optional] minimum number of trajectories to find*
 *    ^max-number - [Optional] maximum number of trajectories to find*
 *    ^min-time - [Optional] minumum seconds to spend planning**
 *    ^max-time - [Optional] maximum seconds to spend planning**
 *    ^target-samples - [Optional] number of IK solutions to explore for target state
 *
 * * After finding the min number of trajectories, the search will continue and
 *   wait to be cut off, although its status will switch to "continuing". After
 *   finding the max number of trajectories, the search will stop and be marked "complete".
 * ** After the min number of seconds, the search will continue and wait to be cut
 *    off with its status set to "continuing". After the max number of seconds, the
 *    search will stop and be marked "complete".
 */

class find_trajectories_command : public command
{
public:
    static const int LIM_INF = -1;
    static int next_id;

    find_trajectories_command(svs_state* state, Symbol* root) : command(state, root),
                                                                root(root),
                                                                fails_root(NULL),
                                                                parsed(false) {
        si = state->get_svs()->get_soar_interface();
        ms = state->get_motor_state();

        // All data about target and search limits are stored in query struct,
        // will be updated with actual values when command is parsed
        search_query.min_num = LIM_INF;
        search_query.max_num = LIM_INF;
        search_query.min_time = LIM_INF;
        search_query.max_time = LIM_INF;

        // Each command will have its own unique int ID
        id = next_id;
        next_id++;

        failure_wmes.resize(NUM_FAILURE_TYPES);
    }

    std::string description() { return "find-trajectories"; }
    int command_type() { return SVS_WRITE_COMMAND; }

    bool update_sub() {
        // When command is first put on the link, parse it
        if (!parsed) {
            parsed = true;
            // If parsed successfully, set the status to "parsed"
            if (parse()) {
                ms->new_query(id, search_query);
                // XXX Does adding id to commands require instantiation?
                si->make_wme(root, "id", si->make_sym(id));
                set_status("parsed");
                prev_status = "parsed";
            } else {
                // Error message already set in parse() method
                return false;
            }
        } else {
            std::string stat = ms->get_query_status(id);

            if (stat != "stopped" && check_stop()) ms->stop_query(id);

            if (stat != prev_status) {
                set_status(stat);
            }
            std::vector<int> fails = ms->get_query_failures(id);
            // if (fails.size() > 0)
            //     std::cout << "START_INVALID " << fails[0]
            //               << " GOAL_INVALID " << fails[1]
            //               << " PLANNING_FAILURE " << fails[2]
            //               << " OTHER_ERROR " << fails[3] << std::endl;
            if (fails.size() > 0 && fails != prev_failures) {
                update_failures(fails);
            }
        }
        return true;
    }

private:
    bool check_stop() {
        std::string val;
        si->get_const_attr(root, "stop", val);

        if (val.empty()) return false;
        return true;
    }

    void update_failures(std::vector<int>& fv) {
        if (!fails_root)
            fails_root = si->get_wme_val(si->make_id_wme(root, "failures"));

        prev_failures.resize(NUM_FAILURE_TYPES);

        for (int i = 0; i < fv.size(); i++) {
            if (fv[i] == prev_failures[i]) continue;
            if (failure_wmes[i]) si->remove_wme(failure_wmes[i]);
            failure_wmes[i] = si->make_wme(fails_root, ft_to_str(int_to_ft(i)), fv[i]);
            prev_failures[i] = fv[i];
        }
    }

    bool parse() {
        std::cout << "Parsing a find-trajectories command!!" << std::endl;

        // ^min-number and ^max-number (can't read them as ints from WM)
        double min_d = 0;
        double max_d = 0;
        si->get_const_attr(root, "min-number", min_d);
        si->get_const_attr(root, "max-number", max_d);
        if (min_d > 0) search_query.min_num = (int) min_d;
        else search_query.min_num = -1;
        if (max_d > 0) search_query.max_num = (int) max_d;
        else search_query.max_num = -1;

        // sanity check
        if (search_query.max_num != -1 && search_query.max_num < search_query.min_num) {
            set_status("max-num must be larger than min-num");
            return false;
        }

        // ^min-time and ^max-time
        si->get_const_attr(root, "min-time", search_query.min_time);
        si->get_const_attr(root, "max-time", search_query.max_time);

        // sanity check
        if (search_query.max_time != -1 && search_query.max_time < search_query.min_time) {
            set_status("max-time must be larger than min-time");
            return false;
        }

        double samples = 0;
        si->get_const_attr(root, "target-samples", samples);
        if (samples > 0) search_query.target_samples = (int) samples;
        else search_query.target_samples = -1;

        // ^target <t>
        wme* target_wme;
        if (!si->find_child_wme(root, "target", target_wme)) {
            set_status("no target found");
            return false;
        }
        target_root = si->get_wme_val(target_wme);

        // <t> ^center
        if (!si->get_vec3(target_root, "center", search_query.target_center)) {
            set_status("no target center found");
            return false;
        }

        // <t> ^frame
        if (!si->get_const_attr(target_root, "frame", search_query.target_frame)) {
            set_status("no target frame found");
            return false;
        }
        if (search_query.target_frame != "self" && search_query.target_frame != "world") {
            set_status("target frame unsupported");
            return false;
        }

        // <t> ^size
        wme* size_wme;
        if (si->find_child_wme(target_root, "size", size_wme)) {
            Symbol* size_root = si->get_wme_val(size_wme);
            wme* x;
            if (si->find_child_wme(size_root, "x", x)) {
                search_query.target_type = BOX_TARGET;
                si->get_vec3(target_root, "size", search_query.target_box_size);
            } else {
                search_query.target_type = SPHERE_TARGET;
                si->get_const_attr(target_root, "size", search_query.target_sphere_radius);
            }
        } else {
            search_query.target_type = POINT_TARGET;
        }

        // <t> ^orientation and orientation-flex
        if (si->get_vec3(target_root, "orientation", search_query.orientation)) {
            search_query.use_orientation = true;
        } else {
            search_query.use_orientation = false;
        }
        if (si->get_const_attr(target_root, "orientation-flex", search_query.orientation_flex)) {
            if (!search_query.use_orientation) {
                set_status("orientation-flex used without orientation");
                return false;
            }
            search_query.use_orientation_flex = true;
        } else {
            search_query.use_orientation_flex = false;
        }

        search_query.joint_group = "arm"; // XXX Possibly make option

        search_query.holding_object = false;
        // XXX add ability to parse held object

        std::cout << "--- COMMAND WITH ID " << id << " ---" << std::endl
                  << search_query.to_str() << std::endl
                  << "--------------------------" << std::endl;

        return true;
    }

    soar_interface* si;
    motor_state* ms;
    Symbol* root;
    Symbol* fails_root;
    std::vector<wme*> failure_wmes;

    bool parsed;
    query search_query;
    std::string prev_status;
    std::vector<int> prev_failures;

    int id;

    Symbol* target_root;
};

int find_trajectories_command::next_id = 0;

command* _make_find_trajectories_command_(svs_state* state, Symbol* root)
{
    return new find_trajectories_command(state, root);
}

command_table_entry* find_trajectories_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "find-trajectories";
    e->description = "Finds trajectories that match the given query";
    e->parameters["target"] = "Spatial target of the trajectories { ^center ^size ^orientation ^orientation-flex }";
    e->parameters["min-number"] = "[Optional] Minimum number of trajectories to find";
    e->parameters["max-number"] = "[Optional] Maximum number of trajectories to find";
    e->parameters["min-time"] = "[Optional] Minimum seconds to spend planning";
    e->parameters["max-time"] = "[Optional] Maximum seconds to spend planning";
    e->create = &_make_find_trajectories_command_;
    return e;
}

#endif
