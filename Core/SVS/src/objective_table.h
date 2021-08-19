#ifndef OBJECTIVE_TABLE_H
#define OBJECTIVE_TABLE_H

#ifdef ENABLE_ROS

#include "cliproxy.h"
#include "soar_interface.h"

//class objective;

class objective_table_entry : public cliproxy {
public:
    objective_table_entry();

    // objective* (*create)(...)

    std::string name;
    std::string description;
    std::map<std::string, std::string> parameters;

    void proxy_use_sub(const std::vector<std::string>& args, std::ostream& os);
};

/*
 * objective_table class
 *
 * Maps the names of objectives to the functions used to create an instances
 * of objectives that can be used to evaluate trajectories (held in the
 * objective_table_entry instances). Should be used as a singleton, accessed through
 * get_objective_table(). Inspired by the command_table and filter_table.
 */

class objective_table : public cliproxy {
public:
    friend objective_table& get_objective_table();
    void del_entries();
    //objective* make_objective() const;

private:
    objective_table();
    void add(objective_table_entry* e);
    void proxy_get_children(std::map<std::string, cliproxy*>& c);
    void proxy_use_sub(const std::vector<std::string>& args, std::ostream& os);

    std::map<std::string, objective_table_entry*> table;
};

// Returns the singleton instance of the table
objective_table& get_objective_table();

#endif
#endif
