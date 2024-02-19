#ifdef ENABLE_ROS

#include "endpoint.h"
#include "objective_table.h"
#include "motor_state.h"
#include "motor/motor.h"

/////////////////////////////// CTR //////////////////////////////////////

centrality_objective::centrality_objective(Symbol* cmd_rt,
                                           soar_interface* si,
                                           motor_state* ms,
                                           objective_input* oi) : objective(cmd_rt,
                                                                            si,
                                                                            ms,
                                                                            oi),
                                                                  mtr(ms->get_motor())
{
    if (input->count("area-center")) {
        filter_val_c<vec3>* ctr_fval =
            static_cast<filter_val_c<vec3>*>((*input)["area-center"]);
        center_pt = ctr_fval->get_value();
    } else {
        std::cout << "[Warning] No center provided to centrality objective!" << std::endl;
    }

    bool is_box = false;
    if (input->count("area-dimensions")) {
        is_box = true;
        filter_val_c<vec3>* dim_fval =
            static_cast<filter_val_c<vec3>*>((*input)["area-dimensions"]);
        vec3 box_dim = dim_fval->get_value();
        max_dist = (box_dim / 2.0).norm(); // distance from center to one of the corners
    }

    bool is_sphere = false;
    if (input->count("area-radius")) {
        is_sphere = true;
        filter_val_c<double>* rad_fval =
            static_cast<filter_val_c<double>*>((*input)["area-radius"]);
        max_dist = rad_fval->get_value();
    }

    if (!is_box && !is_sphere) {
        std::cout << "[Warning] Centrality area has no valid dimensions!" << std::endl;
    }
}

double centrality_objective::evaluate_on(trajectory& t) {
    // Get last waypoint ee xyz
    std::map<std::string, double> end_state;
    std::vector<std::string>::iterator n = t.joints.begin();
    int m = 0;
    for (; n != t.joints.end(); n++) {
        end_state[*n] = t.waypoints[t.length-1][m];
        m++;
    }
    std::map<std::string, double>::iterator f = t.fixed_joints.begin();
    for (; f != t.fixed_joints.end(); f++) {
        end_state[f->first] = f->second;
    }

    transform3 ee_end = mtr->get_ee_frame_transform_at(end_state);
    vec3 ee_pos;
    ee_end.position(ee_pos);

    double dist = (center_pt - ee_pos).norm();

    //std::cout << "Endpoint [" << ee_pos[0] << ", " << ee_pos[1] << ", "
    //          << ee_pos[2] << "] = " << dist/max_dist << " centrality value" << std::endl;

    return (max_dist - dist)/max_dist;
}

objective* make_centrality_objective(Symbol* cmd_rt,
                                     soar_interface* si,
                                     motor_state* ms,
                                     objective_input* oi) {
    return new centrality_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* centrality_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "endpoint-centrality";
    e->description = "Distance to center of target area";
    e->parameters["set-id"] = "Trajectory set";
    e->parameters["area-center"] = "Center of area of interest";
    e->parameters["area-dimensions"] = "[Optional] Dimensions of a box area";
    e->parameters["area-radius"] = "[Optional] Radius of a sphere area";
    e->create = &make_centrality_objective;
    return e;
}

/////////////////////////////// DTO //////////////////////////////////////

distance_objective::distance_objective(Symbol* cmd_rt,
                                           soar_interface* si,
                                           motor_state* ms,
                                           objective_input* oi) : objective(cmd_rt,
                                                                            si,
                                                                            ms,
                                                                            oi),
                                                                  mtr(ms->get_motor())
{
    has_valid_obj = false;
    std::string obj_name;

    if (input->count("obstacles")) {
        filter_val_c<std::string>* names_str =
            dynamic_cast<filter_val_c<std::string>*>((*input)["obstacles"]);
        std::string all_obs = names_str->get_value();

        if (!all_obs.empty() && all_obs.find(" ") != std::string::npos) {
            obj_name = all_obs.substr(0, all_obs.find(" "));
            std::cout << "[Warning] Too many obstacles provided to distance objective!"
                      << " Ignoring extra obstacles."
                      << std::endl;
        } else {
            obj_name = all_obs;
        }
        has_valid_obj = true;
    } else {
        std::cout << "[Warning] No object provided to distance objective!"
                  << std::endl;
    }

    if (!has_valid_obj) return;

    std::vector<obstacle> obstacles;
    ms->get_scene_obstacles(obstacles);

    bool found = false;
    std::vector<obstacle>::iterator o = obstacles.begin();
    for (; o != obstacles.end(); o++) {
        if (o->name == obj_name) {
            found = true;
            object = *o;
            break;
        }
    }

    if (!found) {
        has_valid_obj = false;
        std::cout << "[Warning] Requesting distance objective for nonexistent object "
                  << obj_name << std::endl;
        return;
    }
}

double distance_objective::evaluate_on(trajectory& t) {
    // Get last waypoint ee xyz
    std::map<std::string, double> end_state;
    std::vector<std::string>::iterator n = t.joints.begin();
    int m = 0;
    for (; n != t.joints.end(); n++) {
        end_state[*n] = t.waypoints[t.length-1][m];
        m++;
    }
    std::map<std::string, double>::iterator f = t.fixed_joints.begin();
    for (; f != t.fixed_joints.end(); f++) {
        end_state[f->first] = f->second;
    }

    transform3 ee_end = mtr->get_ee_frame_transform_at(end_state);
    vec3 ee_pos;
    ee_end.position(ee_pos);

    vec3 obj_pos;
    object.transform.position(obj_pos);

    double dist = (obj_pos - ee_pos).norm();

    return dist;
}

objective* make_distance_objective(Symbol* cmd_rt,
                                     soar_interface* si,
                                     motor_state* ms,
                                     objective_input* oi) {
    return new distance_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* distance_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "endpoint-distance";
    e->description = "Distance to center of target area";
    e->parameters["set-id"] = "Trajectory set";
    e->parameters["area-center"] = "Center of area of interest";
    e->parameters["area-dimensions"] = "[Optional] Dimensions of a box area";
    e->parameters["area-radius"] = "[Optional] Radius of a sphere area";
    e->create = &make_distance_objective;
    return e;
}

#endif
