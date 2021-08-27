#ifndef OBJECTIVE_H
#define OBJECTIVE_H

#ifdef ENABLE_ROS

#include <string>
#include <map>

#include "soar_interface.h"
#include "filter_val.h"

typedef std::map<std::string, filter_val*> objective_input;
class motor_state;

enum OutputType {
    VALUE, // ^objective_name <value>
    RANK, // ^objective_name-rank <rank>
    SELECT // ^objective_name-selected <yes/no>
};

class objective {
public:
    objective(Symbol* cmd_rt,
              soar_interface* si,
              motor_state* ms,
              objective_input* oi);
    virtual ~objective();

    // Evaluate the objective with results -> values
    virtual bool evaluate() = 0;
    // Turn the values -> outputs based on the OutputType
    void update_outputs();
    OutputType output_type() { return ot; }

    // Put a status on the command object
    void set_status(const std::string& msg);

protected:
    Symbol* cmd_rt;
    soar_interface* si;
    std::string status;
    wme* status_wme;

    objective_input* input;
    motor_state* ms;
    int set_id;
    int subset_size;

    std::map<int, double> values;
    OutputType ot;
    std::map<int, double> outputs;
};

#endif
#endif
