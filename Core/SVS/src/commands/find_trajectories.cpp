#ifdef ENABLE_ROS

#include "svs.h"
#include "symbol.h"
#include "command.h"
#include "command_table.h"
#include "scene.h"
#include "trajectory_set.h"


/*
 * find_trajectories_command class
 *
 * Defines the Soar interface for requesting and controlling the
 * creation of a MAST trajectory set through a ^find-trajectoreis command.
 *
 * Usage:
 *    ^target <t> - gripper target area for the motion
 *        ^center <vec3> - center of the target area
 *        ^size <vec3> OR <double> - if vec3, half box dimensions; if double, radius
 *        ^orientation <vec3> - [Optional] gripper orientation rpy
 *        ^orientation-flex <vec3> - [Optional] accpetable variation from target orientation
 *    ^min-number - [Optional] minimum number of trajectories to find*
 *    ^max-number - [Optional] maximum number of trajectories to find*
 *    ^min-time - [Optional] minumum seconds to spend planning**
 *    ^max-time - [Optional] maximum seconds to spend planning**
 *
 * * After finding the min number of trajectories, the search will continue and
 *   wait to be cut off, although its status will switch to running_complete. After
 *   finding the max number of trajectories, the search will stop and be marked complete.
 * ** After the min number of seconds, the search will continue and wait to be cut
 *    off with its status set to running_complete. After the max number of seconds, the
 *    search will stop and be marked complete.
 */

enum TargetType{
    POINT_TARGET,
    BOX_TARGET,
    SPHERE_TARGET
};

class find_trajectories_command : public command
{
public:
    static const int LIM_INF = -1;

    find_trajectories_command(svs_state* state, Symbol* root) : command(state, root),
                                                                root(root),
                                                                parsed(false),
                                                                min_num(LIM_INF),
                                                                max_num(LIM_INF),
                                                                min_time(LIM_INF),
                                                                max_time(LIM_INF) {
        si = state->get_svs()->get_soar_interface();
    }

    std::string description() { return "find-trajectories"; }
    int command_type() { return SVS_WRITE_COMMAND; }

    bool update_sub() {
        // When command is first put on the link, parse it
        if (!parsed) {
            parsed = true;
            return parse();
        }
        return true;
    }

private:
    bool parse() {
        std::cout << "Parsing a find-trajectories command!!" << std::endl;

        // ^min-number and ^max-number (can't read them as ints from WM)
        double min_d, max_d;
        si->get_const_attr(root, "min-number", min_d);
        si->get_const_attr(root, "max-number", max_d);
        if (min_d > 0) min_num = (int) min_d;
        if (max_d > 0) max_num = (int) max_d;

        // ^min-time and ^max-time
        si->get_const_attr(root, "min-time", min_time);
        si->get_const_attr(root, "max-time", max_time);

        // ^target <t>
        wme* target_wme;
        if (!si->find_child_wme(root, "target", target_wme)) {
            set_status("no target found");
            return false;
        }
        target_root = si->get_wme_val(target_wme);

        // <t> ^center
        if (!si->get_vec3(target_root, "center", target_center)) {
            set_status("no target center found");
            return false;
        }

        // <t> ^size
        wme* size_wme;
        if (si->find_child_wme(target_root, "size", size_wme)) {
            Symbol* size_root = si->get_wme_val(size_wme);
            wme* x;
            if (si->find_child_wme(size_root, "x", x)) {
                target_type = BOX_TARGET;
                si->get_vec3(target_root, "size", target_box_size);
            } else {
                target_type = SPHERE_TARGET;
                si->get_const_attr(target_root, "size", target_sphere_radius);
            }
        } else {
            target_type = POINT_TARGET;
        }

        // <t> ^orientation and orientation-flex
        if (si->get_vec3(target_root, "orientation", orientation)) {
            use_orientation = true;
        } else {
            use_orientation = false;
        }
        if (si->get_vec3(target_root, "orientation-flex", orientation_flex)) {
            if (!use_orientation) {
                set_status("orientation-flex used without orientation");
                return false;
            }
            use_orientation_flex = true;
        } else {
            use_orientation_flex = false;
        }

        std::cout << "Min num: " << min_num << " Max num: " << max_num << std::endl
                  << "Min time: " << min_time << " Max time: " << max_time << std::endl
                  << "Target: " << target_center[0] << ", " << target_center[1] << ", "
                  << target_center[2] << std::endl << "Target type: " << target_type
                  << std::endl;

        return true;
    }

    soar_interface* si;
    Symbol* root;

    bool parsed;

    int min_num;
    int max_num;
    double min_time;
    double max_time;

    Symbol* target_root;

    vec3 target_center;
    TargetType target_type;
    vec3 target_box_size; // used for BOX_TARGET
    double target_sphere_radius; // used for SPHERE_TARGET

    bool use_orientation;
    vec3 orientation;
    bool use_orientation_flex;
    vec3 orientation_flex;
};

command* _make_find_trajectories_command_(svs_state* state, Symbol* root)
{
    return new find_trajectories_command(state, root);
}

command_table_entry* find_trajectories_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "find-trajectories";
    e->description = "Add trajectories to the trajectory_set";
    e->parameters["target"] = "Spatial target of the trajectories { ^center ^size ^orientation ^orientation-flex }";
    e->parameters["min-number"] = "[Optional] Minimum number of trajectories to find";
    e->parameters["max-number"] = "[Optional] Maximum number of trajectories to find";
    e->parameters["min-time"] = "[Optional] Minimum seconds to spend planning";
    e->parameters["max-time"] = "[Optional] Maximum seconds to spend planning";
    e->create = &_make_find_trajectories_command_;
    return e;
}

#endif
