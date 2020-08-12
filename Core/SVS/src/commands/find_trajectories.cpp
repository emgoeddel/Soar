#ifdef ENABLE_ROS

#include "svs.h"
#include "symbol.h"
#include "command.h"
#include "command_table.h"
#include "scene.h"
#include "trajectory_set.h"

class find_trajectories_command : public command
{
public:
    find_trajectories_command(svs_state* state, Symbol* root)
        : command(state, root) {}
    std::string description() { return "find_trajectories"; }
    int command_type() { return SVS_WRITE_COMMAND; }

    bool update_sub() { return true; }

private:
    bool parse() { return true; }
};

command* _make_find_trajectories_command_(svs_state* state, Symbol* root)
{
    return new find_trajectories_command(state, root);
}

command_table_entry* find_trajectories_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "find_trajectories";
    e->description = "Add trajectories to the trajectory_set";
    e->parameters["target"] = "Target of the trajectories";
    e->parameters["min_number"] = "Minimum number of trajectories to find";
    e->parameters["max_number"] = "Maximum number of trajectories to find";
    e->parameters["min_time"] = "Minimum seconds to spend planning";
    e->parameters["max_time"] = "Maximum seconds to spend planning";
    e->create = &_make_find_trajectories_command_;
    return e;
}

#endif
