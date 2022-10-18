#ifdef ENABLE_ROS

#include "relative.h"
#include "objective_table.h"
#include "motor/motor.h"
#include "motor_state.h"

/////////////////////////////// PTO //////////////////////////////////////
proportion_over_objective::proportion_over_objective(Symbol* cmd_rt,
                                                     soar_interface* si,
                                                     motor_state* ms,
                                                     objective_input* oi) :
    objective(cmd_rt, si, ms, oi),
    has_valid_obj(false) {
    std::shared_ptr<motor> mtr = ms->get_motor();
    if (input->count("obstacles")) {
        filter_val_c<std::string>* names_str =
            dynamic_cast<filter_val_c<std::string>*>((*input)["obstacles"]);
        std::string all_obs = names_str->get_value();

        if (!all_obs.empty() && all_obs.find(" ") != std::string::npos) {
            obj_name = all_obs.substr(0, all_obs.find(" "));
            std::cout << "[Warning] Too many obstacles provided to proportion-over objective!"
                      << " Ignoring extra obstacles."
                      << std::endl;
        } else {
            obj_name = all_obs;
        }
        has_valid_obj = true;
    } else {
        std::cout << "[Warning] No object provided to proportion-over objective!"
                  << std::endl;
    }

    if (!has_valid_obj) return;

    std::vector<obstacle> obstacles;
    ms->get_scene_obstacles(obstacles);

    transform3 xform;
    vec3 dim;
    bool found = false;
    std::vector<obstacle>::iterator o = obstacles.begin();
    for (; o != obstacles.end(); o++) {
        if (o->name == obj_name) {
            found = true;
            if (o->geometry != BOX_OBSTACLE) {
                std::cout << "[Warning] Non-box obstacle requested in proportion-over objective!"
                          << std::endl;
                has_valid_obj = false;
                return;
            }
            xform = o->transform;
            dim = o->box_dim;
            break;
        }
    }

    if (!found) {
        has_valid_obj = false;
        std::cout << "[Warning] Requesting proportion-over objective for nonexistent object "
                  << obj_name << std::endl;
        return;
    }

    obstacle over_space;
    over_space.name = "over_space";
    over_space.geometry = BOX_OBSTACLE;

    vec3 over_pos;
    vec3 over_rot;
    xform.position(over_pos);
    over_pos[2] += ((dim[2] * 0.5) + 0.5); // Shift up in z
    xform.rotation(over_rot);
    over_rot[0] = 0.0;
    over_rot[1] = 0.0; // Ignore any roll and pitch (assume small)

    over_space.transform = transform3(over_pos, over_rot, vec3(1, 1, 1));
    over_space.box_dim = dim;
    over_space.box_dim[2] = 1.0; // Arbitrarily set as 1m space above object

    std::vector<obstacle> over_only;
    over_only.push_back(over_space);

    cc = mtr->build_collision_checker(ms->get_base_xform(),
                                      ms->get_joints(),
                                      over_only);

    //////// DBG ///////
    // std::map<std::string, double> joints = ms->get_joints();
    // cc->print_scene(joints);
    ///////////////////

}

proportion_over_objective::~proportion_over_objective() {
    delete cc;
}

double proportion_over_objective::evaluate_on(trajectory& t) {
    if (!has_valid_obj) return 0;

    int num_over = 0;
    std::vector<std::vector<double> >::iterator w = t.waypoints.begin();
    for (; w != t.waypoints.end(); w++) {
        if (!cc->is_valid(*w)) num_over++;
    }

    return ((double) num_over / (double) t.length);
}

objective* make_proportion_over_objective(Symbol* cmd_rt,
                                           soar_interface* si,
                                           motor_state* ms,
                                           objective_input* oi) {
    return new proportion_over_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* proportion_over_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "proportion-over";
    e->description = "Proportion of trajectory with arm over obstacle";
    e->parameters["set-id"] = "Trajectory set";
    e->parameters["obstacle"] = "Object of interest";
    e->create = &make_proportion_over_objective;
    return e;
}

#endif
