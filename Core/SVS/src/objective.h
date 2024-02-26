#ifndef OBJECTIVE_H
#define OBJECTIVE_H

#ifdef ENABLE_ROS

#include <string>
#include <map>

#include "soar_interface.h"
#include "motor_types.h"
#include "filter_val.h"

typedef std::map<std::string, filter_val*> objective_input;
class motor_state;

enum OutputType {
    VALUE, // ^objective_name <value>
    RANK, // ^objective_name-rank <rank>
    SELECT // ^selected-by objective_name (or not)
};

class objective {
public:
    objective(Symbol* cmd_rt,
              soar_interface* si,
              motor_state* ms,
              objective_input* oi);
    virtual ~objective();

    // Evaluate the objective with results -> values
    virtual double evaluate_on(trajectory& t) = 0;
    bool evaluate();
    void get_latest_trajectories();
    bool update_outputs();
    OutputType output_type() { return ot; }

    std::string get_name() { return name; }
    std::map<int, double> get_outputs() { return outputs; }
    int get_selected();

    // Put a status on the command object
    void set_status(const std::string& msg);

protected:
    OutputType str_to_output(std::string s);

    Symbol* cmd_rt;
    soar_interface* si;
    std::string name;
    std::string status;
    wme* status_wme;

    objective_input* input;
    motor_state* ms;
    int set_id;
    std::map<int, trajectory> trajectories;
    std::set<int> prev_selected;
    int subset_size;
    std::string subset_type;
    bool maximize;

    std::map<int, double> values;
    OutputType ot;
    std::map<int, double> outputs;
};

#endif
#endif
