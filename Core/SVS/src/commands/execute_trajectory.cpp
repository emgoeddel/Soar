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
