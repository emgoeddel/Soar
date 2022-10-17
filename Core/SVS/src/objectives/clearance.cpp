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

double min_clearance_objective::evaluate_on(trajectory& t) {
    double min_clear = 1000;
    std::vector<std::vector<double> >::iterator w = t.waypoints.begin();
    for (; w != t.waypoints.end(); w++) {
        double clr = cc->minimum_distance(*w);
        if (clr < min_clear) min_clear = clr;
    }

    return min_clear;
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
    e->description = "Minimum distance to all obstacles across trajectory";
    e->parameters["set-id"] = "Trajectory set";
    e->create = &make_min_clearance_objective;
    return e;
}

/////////////////////////////// MCS //////////////////////////////////////
min_clear_subset_objective::min_clear_subset_objective(Symbol* cmd_rt,
                                                       soar_interface* si,
                                                       motor_state* ms,
                                                       objective_input* oi) :
    objective(cmd_rt, si, ms, oi) {
    std::shared_ptr<motor> mtr = ms->get_motor();

    filter_val_c<std::string>* names_str =
        dynamic_cast<filter_val_c<std::string>*>((*input)["obstacles"]);
    std::string all_obs = names_str->get_value();

    // XXX Hack because the obstacle names are all passed in one string
    std::vector<std::string> subset_names;
    int p = 0;
    while (all_obs.find(" ", p) != std::string::npos) {
        int nxt_sp = all_obs.find(" ", p);
        subset_names.push_back(all_obs.substr(p, (nxt_sp-p)));
        p = nxt_sp + 1;
    }

    std::vector<obstacle> obstacles;
    ms->get_scene_obstacles(obstacles);

    std::vector<obstacle> subset;
    std::vector<std::string>::iterator s = subset_names.begin();
    for (; s != subset_names.end(); s++) {
        bool match = false;
        std::vector<obstacle>::iterator o = obstacles.begin();
        for (; o != obstacles.end(); o++) {
            if (*s == o->name) {
                subset.push_back(*o);
                match = true;
                break;
            }
        }
        if (match) continue;
        std::cout << "[Warning] Requesting distance from non-existing obstacle "
                  << *s << "!" << std::endl;
    }

    cc = mtr->build_collision_checker(ms->get_base_xform(),
                                      ms->get_joints(),
                                      subset);

    //////// DBG ///////
    // ompl::base::ScopedState<> ompl_state(cc->get_space());
    // std::map<std::string, double> joints = ms->get_joints();
    // ompl_state[0] = joints["shoulder_pan_joint"];
    // ompl_state[1] = joints["shoulder_lift_joint"];
    // ompl_state[2] = joints["upperarm_roll_joint"];
    // ompl_state[3] = joints["elbow_flex_joint"];
    // ompl_state[4] = joints["forearm_roll_joint"];
    // ompl_state[5] = joints["wrist_flex_joint"];
    // ompl_state[6] = joints["wrist_roll_joint"];
    // cc->print_scene(ompl_state.get());
    ///////////////////
}

min_clear_subset_objective::~min_clear_subset_objective() {
    delete cc;
}

double min_clear_subset_objective::evaluate_on(trajectory& t) {
    double min_clear = 1000;
    std::vector<std::vector<double> >::iterator w = t.waypoints.begin();
    for (; w != t.waypoints.end(); w++) {
        double clr = cc->minimum_distance(*w);
        if (clr < min_clear) min_clear = clr;
    }

    return min_clear;
}

objective* make_min_clear_subset_objective(Symbol* cmd_rt,
                                        soar_interface* si,
                                        motor_state* ms,
                                        objective_input* oi) {
    return new min_clear_subset_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* min_clear_subset_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "min-clear-subset";
    e->description = "Minimum distance to a specified subset of obstacles across trajectory";
    e->parameters["set-id"] = "Trajectory set";
    e->parameters["obstacles"] = "Set of obstacles to consider";
    e->create = &make_min_clear_subset_objective;
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

double weighted_avg_clearance_objective::evaluate_on(trajectory& t) {
    double cost_total = 0;

    std::vector<std::vector<double> >::iterator w = t.waypoints.begin();
    for (; w != t.waypoints.end(); w++) {
        double clr = cc->minimum_distance(*w);
        if (clr > MAX_CLR_FOR_AVG) continue;
        if (clr < 0) { // XXX Collision; could penalize more?
            cost_total += 1;
            continue;
        }
        cost_total += ((MAX_CLR_FOR_AVG - clr) / MAX_CLR_FOR_AVG);
    }

    return (cost_total / (double)t.length);
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
