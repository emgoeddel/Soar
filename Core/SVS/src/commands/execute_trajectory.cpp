#ifdef ENABLE_ROS

#include "svs.h"
#include "symbol.h"
#include "command.h"
#include "command_table.h"
#include "scene.h"
#include "motor_state.h"
#include "objective_table.h" // for dbg only

#include <time.h> // demo output

class execute_trajectory_command : public command {
public:
    execute_trajectory_command(svs_state* state, Symbol* root) : command(state, root),
                                                                 root(root),
                                                                 parsed(false) {
        si = state->get_svs()->get_soar_interface();
        ri = state->get_svs()->get_ros_interface();
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
            std::chrono::duration<double> et = std::chrono::system_clock::now() - start_time;
            std::cout << "Execution of trajectory took "
                      << et.count() << "s in wall time." << std::endl;

            last_status = "finished";
            set_status(last_status);
            si->make_wme(root, "result", ri->execution_result());
        }

        return true;
    }

private:
    bool parse() {
        std::cout << "Parsing an execute-trajectory command!!" << std::endl;

        // XXX Add this check in future when agent can manipulate robot hypothetically
        // Don't let the agent exec if this state's joints aren't the robot's
        // actual current joints
        // if (ms->get_joints_type() != "current") {
        //     set_status("cannot execute from hypothetical joint state");
        //     return false;
        // }

        // Get the set-id from the command and error out if it's not present
        wme* set_wme;
        int set_id;
        if (!si->find_child_wme(root, "set-id", set_wme)) {
            set_status("no set-id found");
            return false;
        } else {
            double tmp;
            si->get_const_attr(root, "set-id", tmp);
            set_id = (int) tmp;
        }

        // Get the trajectory-id from the command and error out if it's not present
        wme* traj_id_wme;
        int traj_id;
        if (!si->find_child_wme(root, "trajectory-id", traj_id_wme)) {
            set_status("no trajectory-id found");
            return false;
        } else {
            double tmp;
            si->get_const_attr(root, "trajectory-id", tmp);
            traj_id = (int) tmp;
        }

        // Use ids to figure out which underlying trajectory this
        trajectory t;
        if (!ms->match_trajectory(set_id, traj_id, t)) {
            set_status("no corresponding trajectory in set");
            return false;
        }

        // Add check that trajectory's first state is pretty close to current
        // joint state.
        if (!ms->is_start_state_for(t)) {
            set_status("joint state does not match trajectory start");
            return false;
        }

        std::cout << "Executing trajectory " << traj_id << " from set " << set_id
                  << " with length " << t.length << std::endl;
        start_time = std::chrono::system_clock::now();
        ri->send_trajectory(t);

        // EVAL
        if (ms->do_output()) {
            std::ofstream df2;
            df2.open("selections.txt", std::ios::out | std::ios::app);
            if (!df2.is_open()) std::cout << "ERROR writing to file!" << std::endl;
            df2 << traj_id << std::endl;
            df2.close();
        }
        // END EVAL

        ////////////////// FOR EVAL + DEBUGGING //////////////////////
        // objective_input mca;
        // mca["output-type"] = new filter_val_c<std::string>("value");
        // mca["name"] = new filter_val_c<std::string>("min-clearance");
        // mca["number"] = new filter_val_c<int>(1);
        // mca["maximize"] = new filter_val_c<bool>(true);
        // mca["set-id"] = new filter_val_c<int>(set_id);
        // objective* mca_obj = get_objective_table().make_objective("min-clearance",
        //                                                           root,
        //                                                           si,
        //                                                           ms,
        //                                                           &mca);
        // double mca_val = mca_obj->evaluate_on(t);

        // objective_input wac;
        // wac["output-type"] = new filter_val_c<std::string>("value");
        // wac["name"] = new filter_val_c<std::string>("weighted-avg-clearance");
        // wac["number"] = new filter_val_c<int>(1);
        // wac["maximize"] = new filter_val_c<bool>(true);
        // wac["set-id"] = new filter_val_c<int>(set_id);
        // objective* wac_obj = get_objective_table().make_objective("weighted-avg-clearance",
        //                                                           root,
        //                                                           si,
        //                                                           ms,
        //                                                           &wac);
        // double wac_val = wac_obj->evaluate_on(t);

        // objective_input aet;
        // aet["output-type"] = new filter_val_c<std::string>("value");
        // aet["name"] = new filter_val_c<std::string>("execution-time");
        // aet["number"] = new filter_val_c<int>(1);
        // aet["maximize"] = new filter_val_c<bool>(true);
        // aet["set-id"] = new filter_val_c<int>(set_id);
        // objective* aet_obj = get_objective_table().make_objective("execution-time",
        //                                                           root,
        //                                                           si,
        //                                                           ms,
        //                                                           &aet);
        // double aet_val = aet_obj->evaluate_on(t);

        // objective_input tjm;
        // tjm["output-type"] = new filter_val_c<std::string>("value");
        // tjm["name"] = new filter_val_c<std::string>("total-joint-movement");
        // tjm["number"] = new filter_val_c<int>(1);
        // tjm["maximize"] = new filter_val_c<bool>(true);
        // tjm["set-id"] = new filter_val_c<int>(set_id);
        // objective* tjm_obj = get_objective_table().make_objective("total-joint-movement",
        //                                                           root,
        //                                                           si,
        //                                                           ms,
        //                                                           &tjm);
        // double tjm_val = tjm_obj->evaluate_on(t);

        // objective_input les;
        // les["output-type"] = new filter_val_c<std::string>("value");
        // les["name"] = new filter_val_c<std::string>("end-effector-length");
        // les["number"] = new filter_val_c<int>(1);
        // les["maximize"] = new filter_val_c<bool>(true);
        // les["set-id"] = new filter_val_c<int>(set_id);
        // objective* les_obj = get_objective_table().make_objective("end-effector-length",
        //                                                           root,
        //                                                           si,
        //                                                           ms,
        //                                                           &les);
        // double les_val = les_obj->evaluate_on(t);

        // std::cout << "EXECUTED TRAJECTORY OBJECTIVE VALUES" << std::endl;
        // std::cout << "    MCA: " << mca_val << std::endl;
        // std::cout << "    WAC: " << wac_val << std::endl;
        // std::cout << "    AET: " << aet_val << std::endl;
        // std::cout << "    TJM: " << tjm_val << std::endl;
        // std::cout << "    LES: " << les_val << std::endl;
        ///////////////////////////////////////////////////////////////

        return true;
    }

    soar_interface* si;
    ros_interface* ri;
    motor_state* ms;
    Symbol* root;

    bool parsed;
    std::string last_status;
    std::chrono::time_point<std::chrono::system_clock> start_time;
};

command* _make_execute_trajectory_command_(svs_state* state, Symbol* root)
{
    return new execute_trajectory_command(state, root);
}

command_table_entry* execute_trajectory_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "execute-trajectory";
    e->description = "Executes a selected trajectory";
    e->parameters["set-id"] = "The command-id of the set containing selected trajectory";
    e->parameters["trajectory-id"] = "The id of the selected trajectory";
    e->create = &_make_execute_trajectory_command_;
    return e;
}

#endif
