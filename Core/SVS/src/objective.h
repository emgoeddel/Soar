#ifndef OBJECTIVE_H
#define OBJECTIVE_H

#ifdef ENABLE_ROS

#include <string>
#include <map>

#include "motor_state.h"
#include "filter_val.h"

typedef std::map<std::string, filter_val*> objective_input;

enum OutputType {
    VALUE, // ^objective_name <value>
    RANK, // ^objective_name-rank <rank>
    SELECT // ^objective_name-selected <yes/no>
};

class objective {
public:
    objective(Symbol* cmd_rt, soar_interface* si, motor_state* ms, objective_input* oi);
    virtual ~objective();

    // Evaluate the objective and update the motor state
    void evaluate();

    // Put a status on the command object
    void set_status(const std::string& msg);

private:
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
