#include "objective_table.h"

#include "objectives/length.h"

objective_table_entry::objective_table_entry() : //create(NULL),
                                                 description("") {
    set_help("Reports information about this objective");
}

void objective_table_entry::proxy_use_sub(const std::vector<std::string>& args,
                                          std::ostream& os) {
    os << "Objective: " << name << std::endl;
    os << "  " << description << std::endl;
    os << "  Parameters:" << std::endl;
    std::map<std::string, std::string>::iterator i;
    for (i = parameters.begin(); i != parameters.end(); i++)
    {
        os << "    " << std::setw(15) << std::left << i->first
           << " | " << i->second << std::endl;
    }
}

//////////////////////////////////////////////

// LENGTH
objective_table_entry* state_count_objective_entry();

//////////////////////////////////////////////

void objective_table::del_entries() {
    std::map<std::string, objective_table_entry*>::iterator i = table.begin();
    for (; i != table.end(); i++) {
        delete i->second;
    }
    table.clear();
}

objective* objective_table::make_objective(const std::string& name,
                                           Symbol* cmd_rt,
                                           soar_interface* si,
                                           motor_state* ms,
                                           objective_input* oi) const {
    return new state_count_objective(cmd_rt, si, ms, oi);
}

objective_table::objective_table() {
    set_help("Prints out a list of all objectives.");

    add(state_count_objective_entry());
}

void objective_table::add(objective_table_entry* e) {
    table[e->name] = e;
}

void objective_table::proxy_get_children(std::map<std::string, cliproxy*>& c) {
    std::map<std::string, objective_table_entry*>::iterator i = table.begin();
    for (; i != table.end(); i++) {
        c[i->first] = i->second;
    }
}

void objective_table::proxy_use_sub(const std::vector<std::string>& args,
                                    std::ostream& os) {
    os << "====================== OBJECTIVE TABLE =======================" << std::endl;
    std::map<std::string, objective_table_entry*>::iterator i = table.begin();
    for (; i != table.end(); i++)
    {
        os << "  " << std::setw(22) << std::left << i->first
           << " | " << i->second->description << std::endl;
    }
    os << "==============================================================" << std::endl;
    os << "For specific objective info, use the command 'svs objectives.objective_name'"
       << std::endl;
}

//////////////////////////////////////////////

objective_table& get_objective_table() {
    static objective_table inst;
    return inst;
}
