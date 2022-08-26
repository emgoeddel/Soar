#ifdef ENABLE_ROS

#include "clearance.h"
#include "objective_table.h"
#include "motor/motor.h"
#include "motor_state.h"

/////////////////////////////// MCA //////////////////////////////////////
min_clearance_objective::min_clearance_objective(Symbol* cmd_rt,
                                                 soar_interface* si,
                                                 motor_state* ms,
                                                 objective_input* oi) : objective(cmd_rt,
                                                                                  si,
                                                                                  ms,
                                                                                  oi) {
    std::shared_ptr<motor> mtr = ms->get_motor();

    std::vector<obstacle> obstacles;
    ms->get_scene_obstacles(obstacles);

    cc = mtr->build_collision_checker(ms->get_base_xform(),
                                      ms->get_joints(),
                                      obstacles);
}

min_clearance_objective::~min_clearance_objective() {
    delete cc;
}

bool min_clearance_objective::evaluate() {
    std::map<int, trajectory>::iterator i = trajectories.begin();
    for (; i != trajectories.end(); i++) {
        double min_clear = 1000;
        std::vector<std::vector<double> >::iterator w = i->second.waypoints.begin();
        for (; w != i->second.waypoints.end(); w++) {
            double clr = cc->minimum_distance(*w);
            if (clr < min_clear) min_clear = clr;
        }

        values[i->first] = min_clear;
        std::cout << "Trajectory " << i->first << ": " << values[i->first] << std::endl;
     }

    return true;
}

objective* make_min_clearance_objective(Symbol* cmd_rt,
                                        soar_interface* si,
                                        motor_state* ms,
                                        objective_input* oi) {
    return new min_clearance_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* min_clearance_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "min-clearance";
    e->description = "Minimum distance to obstacle across trajectory";
    e->parameters["set-id"] = "Trajectory set";
    e->create = &make_min_clearance_objective;
    return e;
}

/////////////////////////////// WAC //////////////////////////////////////

weighted_avg_clearance_objective::weighted_avg_clearance_objective(Symbol* cmd_rt,
                                                 soar_interface* si,
                                                 motor_state* ms,
                                                 objective_input* oi) : objective(cmd_rt,
                                                                                  si,
                                                                                  ms,
                                                                                  oi) {
    std::shared_ptr<motor> mtr = ms->get_motor();

    std::vector<obstacle> obstacles;
    ms->get_scene_obstacles(obstacles);

    cc = mtr->build_collision_checker(ms->get_base_xform(),
                                      ms->get_joints(),
                                      obstacles);

    MAX_CLR_FOR_AVG = 0.05; // XXX 5cm, but should probably be input
}

weighted_avg_clearance_objective::~weighted_avg_clearance_objective() {
    delete cc;
}

bool weighted_avg_clearance_objective::evaluate() {
    std::map<int, trajectory>::iterator i = trajectories.begin();
    for (; i != trajectories.end(); i++) {
        double cost_total = 0;

        std::vector<std::vector<double> >::iterator w = i->second.waypoints.begin();
        for (; w != i->second.waypoints.end(); w++) {
            double clr = cc->minimum_distance(*w);
            if (clr > MAX_CLR_FOR_AVG) continue;
            if (clr < 0) { // XXX Collision; could penalize more?
                cost_total += 1;
                continue;
            }
            cost_total += ((MAX_CLR_FOR_AVG - clr) / MAX_CLR_FOR_AVG);
        }

        values[i->first] = (cost_total / (double)i->second.length);
        std::cout << "Trajectory " << i->first << ": " << values[i->first] << std::endl;
     }

    return true;
}

objective* make_weighted_avg_clearance_objective(Symbol* cmd_rt,
                                        soar_interface* si,
                                        motor_state* ms,
                                        objective_input* oi) {
    return new weighted_avg_clearance_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* weighted_avg_clearance_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "weighted-avg-clearance";
    e->description = "Average clearance cost across trajectory";
    e->parameters["set-id"] = "Trajectory set";
    e->create = &make_weighted_avg_clearance_objective;
    return e;
}

#endif
