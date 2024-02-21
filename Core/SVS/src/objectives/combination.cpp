#ifdef ENABLE_ROS

#include "combination.h"
#include "objective_table.h"

/////////////////////////////// LES //////////////////////////////////////

ee_combo_objective::ee_combo_objective(Symbol* cmd_rt,
                                       soar_interface* si,
                                       motor_state* ms,
                                       objective_input* oi) :
    objective(cmd_rt, si, ms, oi),
    len(cmd_rt, si, ms, oi),
    rot(cmd_rt, si, ms, oi)
{
    LENGTH_WEIGHT = 10.0; // Rough guesses based on typical values!
    ROTATION_WEIGHT = 1.0;
}

double ee_combo_objective::evaluate_on(trajectory& t) {
    double l_val = len.evaluate_on(t);
    double r_val = rot.evaluate_on(t);

    return (LENGTH_WEIGHT*l_val + ROTATION_WEIGHT*r_val);
}

objective* make_ee_combo_objective(Symbol* cmd_rt,
                                         soar_interface* si,
                                         motor_state* ms,
                                         objective_input* oi) {
    return new ee_combo_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* ee_combo_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "end-effector-combo";
    e->description = "Linear combination of end effector length and rotation";
    e->parameters["set-id"] = "Trajectory set";
    e->create = &make_ee_combo_objective;
    return e;
}

#endif
