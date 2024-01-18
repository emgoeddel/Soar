#ifdef ENABLE_ROS

#include "visibility.h"
#include "objective_table.h"
#include "motor/motor.h"
#include "motor_state.h"
#include "scene.h"

/////////////////////////////// AOO //////////////////////////////////////
average_occlusion_objective::average_occlusion_objective(Symbol* cmd_rt,
                                                         soar_interface* si,
                                                         motor_state* ms,
                                                         objective_input* oi) :
    objective(cmd_rt, si, ms, oi) {
    std::shared_ptr<motor> mtr = ms->get_motor();

    if (input->count("obstacles")) {
        filter_val_c<std::string>* names_str =
            dynamic_cast<filter_val_c<std::string>*>((*input)["obstacles"]);
        std::string all_obs = names_str->get_value();

        if (!all_obs.empty() && all_obs.find(" ") != std::string::npos) {
            obj_name = all_obs.substr(0, all_obs.find(" "));
            std::cout << "[Warning] Too many obstacles provided to average-occlusion objective!"
                      << " Ignoring extra obstacles."
                      << std::endl;
        } else {
            obj_name = all_obs;
        }

        scene* s = ms->get_scene();
        obj_int = s->get_node(obj_name);
        if (obj_int == NULL) {
            std::cout << "[Warning] Requesting visibility of non-existing object "
                      << obj_name << "!" << std::endl;
        }
        eye = s->get_node("head_tilt_link");

        if (obj_int && eye)
            calc_view_lines(obj_int, eye, views);

        s->get_self_nodes(occluders);
    } else {
        obj_int = NULL;
        std::cout << "[Warning] No object provided to average-occlusion objective!"
                  << std::endl;
    }
}

double average_occlusion_objective::evaluate_on(trajectory& t) {
    return 0;
}

objective* make_average_occlusion_objective(Symbol* cmd_rt,
                                            soar_interface* si,
                                            motor_state* ms,
                                            objective_input* oi) {
    return new average_occlusion_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* average_occlusion_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "avg-occlusion";
    e->description = "Average percent that an object is occluded across trajectory";
    e->parameters["set-id"] = "Trajectory set";
    e->parameters["obstacle"] = "Object of interest";
    e->create = &make_average_occlusion_objective;
    return e;
}

#endif
