#ifdef ENABLE_ROS

#include "visibility.h"
#include "objective_table.h"
#include "motor/motor.h"
#include "motor_state.h"
#include "scene.h"
#include "sgnode.h"
#include "sgnode_algs.h"

/////////////////////////////// Base //////////////////////////////////////
base_vis_objective::base_vis_objective(Symbol* cmd_rt,
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
            std::cout << "[Warning] Too many obstacles provided to visibility objective!"
                      << " Ignoring extra obstacles."
                      << std::endl;
        } else {
            obj_name = all_obs;
        }

        scene* s = ms->get_scene();
        sgnode* geom_oi = s->get_node(obj_name);
        if (geom_oi == NULL) {
            std::cout << "[Warning] Requesting visibility of non-existing object "
                      << obj_name << "!" << std::endl;
        } else {
            has_valid_obj = true;

            box_node* box = static_cast<box_node*>(geom_oi);
            vec3 dim = box->get_dimensions();

            ptlist verts;
            for (int i = -1; i <= 1; i++) {
                if (i == 0) continue;
                for (int j = -1; j <= 1; j++) {
                    if (j == 0) continue;
                    for (int k = -1; k <= 1; k++) {
                        if (k == 0) continue;
                        verts.push_back(vec3(i * dim[0],
                                             j * dim[1],
                                             k * dim[2]));
                    }
                }
            }
            obj_int = new convex_node("oi_convex", verts);
            vec3 p, r, s;
            geom_oi->get_trans(p, r, s);
            obj_int->set_trans(p, r, s);
        }

        eye = s->get_node("head_tilt_link");
        if (obj_int && eye) {
            calc_view_lines(obj_int, eye, views);
        } else {
            std::cout << "[Warning] Skipping view line calculation in visibility objective!"
                      << std::endl;
        }
    } else {
        obj_int = NULL;
        std::cout << "[Warning] No object provided to visibility objective!"
                  << std::endl;
    }

    ////// DBG //////
    // std::vector<view_line>::iterator d = views.begin();
    // for (; d != views.end(); d++) {
    //     vec3 c = d->first->get_centroid();
    //     std::cout << "View line centroid: ("
    //               << c[0] << ", "
    //               << c[1] << ", "
    //               << c[2] << ")" << std::endl;
    // }
    /////////////

    arm = ms->get_arm_link_names();
}

base_vis_objective::~base_vis_objective() {
    delete obj_int;
}

/////////////////////////////// AOO //////////////////////////////////////
average_occlusion_objective::average_occlusion_objective(Symbol* cmd_rt,
                                                         soar_interface* si,
                                                         motor_state* ms,
                                                         objective_input* oi) :
    base_vis_objective(cmd_rt, si, ms, oi) {}

double average_occlusion_objective::evaluate_on(trajectory& t) {
    if (!has_valid_obj) return 0;

    // Box dimensions stay the same across the trajectory
    std::map<std::string, vec3> boxes = ms->get_link_boxes();
    std::map<std::string, convex_node*> nodes;
    std::map<std::string, vec3>::iterator b = boxes.begin();
    for (; b != boxes.end(); b++) {
        // Ignore links that aren't arm
        bool include = false;
        std::vector<std::string>::iterator a = arm.begin();
        for (; a != arm.end(); a++) {
            if (b->first == *a) {
                 include = true;
                 break;
             }
        }
        if (!include) continue;

        ptlist verts;
        for (int i = -1; i <= 1; i++) {
            if (i == 0) continue;
            for (int j = -1; j <= 1; j++) {
                if (j == 0) continue;
                for (int k = -1; k <= 1; k++) {
                    if (k == 0) continue;
                    verts.push_back(vec3(i * b->second[0],
                                         j * b->second[1],
                                         k * b->second[2]));
                }
            }
        }
        nodes[b->first] = new convex_node(b->first, verts);
    }

    transform3 base = ms->get_base_xform();
    double occlusion_total = 0;

    // Iterate through waypoints
    std::vector<std::vector<double> >::iterator w = t.waypoints.begin();
    for (; w != t.waypoints.end(); w++) {
        std::map<std::string, double> joint_map;
        for (int i = 0; i < w->size(); i++) {
            joint_map[t.joints[i]] = (*w)[i];
        }

        // Need to get new transforms for each link box at each waypoint
        std::vector<const sgnode*> occluders;

        std::map<std::string, transform3> wp_xf = ms->get_link_transforms_at(joint_map);
        std::map<std::string, convex_node*>::iterator n = nodes.begin();
        for (; n != nodes.end(); n++) {
            transform3 world = base * wp_xf[n->first];
            vec3 p, r, s;
            world.position(p);
            world.rotation(r);
            world.scale(s);
            n->second->set_trans(p, r, s);
            occluders.push_back(n->second);
        }

        double co =  convex_occlusion(views, occluders);
        occlusion_total += co;
    }

    std::map<std::string, convex_node*>::iterator d = nodes.begin();
    for (; d != nodes.end(); d++) {
        delete d->second;
    }
    return (occlusion_total / t.length);
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

/////////////////////////////// PRO //////////////////////////////////////
proportion_occlusion_objective::proportion_occlusion_objective(Symbol* cmd_rt,
                                                   soar_interface* si,
                                                   motor_state* ms,
                                                   objective_input* oi) :
    base_vis_objective(cmd_rt, si, ms, oi) {}

double proportion_occlusion_objective::evaluate_on(trajectory& t) {
    if (!has_valid_obj) return 0;

    // Box dimensions stay the same across the trajectory
    std::map<std::string, vec3> boxes = ms->get_link_boxes();
    std::map<std::string, convex_node*> nodes;
    std::map<std::string, vec3>::iterator b = boxes.begin();
    for (; b != boxes.end(); b++) {
        // Ignore links that aren't arm
        bool include = false;
        std::vector<std::string>::iterator a = arm.begin();
        for (; a != arm.end(); a++) {
            if (b->first == *a) {
                 include = true;
                 break;
             }
        }
        if (!include) continue;

        ptlist verts;
        for (int i = -1; i <= 1; i++) {
            if (i == 0) continue;
            for (int j = -1; j <= 1; j++) {
                if (j == 0) continue;
                for (int k = -1; k <= 1; k++) {
                    if (k == 0) continue;
                    verts.push_back(vec3(i * b->second[0],
                                         j * b->second[1],
                                         k * b->second[2]));
                }
            }
        }
        nodes[b->first] = new convex_node(b->first, verts);
    }

    transform3 base = ms->get_base_xform();
    double occluded_states = 0;

    // Iterate through waypoints
    std::vector<std::vector<double> >::iterator w = t.waypoints.begin();
    for (; w != t.waypoints.end(); w++) {
        std::map<std::string, double> joint_map;
        for (int i = 0; i < w->size(); i++) {
            joint_map[t.joints[i]] = (*w)[i];
        }

        // Need to get new transforms for each link box at each waypoint
        std::vector<const sgnode*> occluders;

        std::map<std::string, transform3> wp_xf = ms->get_link_transforms_at(joint_map);
        std::map<std::string, convex_node*>::iterator n = nodes.begin();
        for (; n != nodes.end(); n++) {
            transform3 world = base * wp_xf[n->first];
            vec3 p, r, s;
            world.position(p);
            world.rotation(r);
            world.scale(s);
            n->second->set_trans(p, r, s);
            occluders.push_back(n->second);
        }

        double co =  convex_occlusion(views, occluders);
        if (co >= 0.25) { // 2 out of 8 box corners
            occluded_states++;
        }
    }

    std::map<std::string, convex_node*>::iterator d = nodes.begin();
    for (; d != nodes.end(); d++) {
        delete d->second;
    }

    // std::cout << "Occluded for " << occluded_states << " out of "
    //          << t.length << std::endl;
    return (occluded_states/t.length);
}

objective* make_proportion_occlusion_objective(Symbol* cmd_rt,
                                            soar_interface* si,
                                            motor_state* ms,
                                            objective_input* oi) {
    return new proportion_occlusion_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* proportion_occlusion_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "proportion-occluded";
    e->description = "Proportion of trajectory object is occluded";
    e->parameters["set-id"] = "Trajectory set";
    e->parameters["obstacle"] = "Object of interest";
    e->create = &make_proportion_occlusion_objective;
    return e;
}

/////////////////////////////// OTO //////////////////////////////////////
occlusion_time_objective::occlusion_time_objective(Symbol* cmd_rt,
                                                   soar_interface* si,
                                                   motor_state* ms,
                                                   objective_input* oi) :
    base_vis_objective(cmd_rt, si, ms, oi) {}

double occlusion_time_objective::evaluate_on(trajectory& t) {
    if (!has_valid_obj) return 0;

    // Box dimensions stay the same across the trajectory
    std::map<std::string, vec3> boxes = ms->get_link_boxes();
    std::map<std::string, convex_node*> nodes;
    std::map<std::string, vec3>::iterator b = boxes.begin();
    for (; b != boxes.end(); b++) {
        // Ignore links that aren't arm
        bool include = false;
        std::vector<std::string>::iterator a = arm.begin();
        for (; a != arm.end(); a++) {
            if (b->first == *a) {
                 include = true;
                 break;
             }
        }
        if (!include) continue;

        ptlist verts;
        for (int i = -1; i <= 1; i++) {
            if (i == 0) continue;
            for (int j = -1; j <= 1; j++) {
                if (j == 0) continue;
                for (int k = -1; k <= 1; k++) {
                    if (k == 0) continue;
                    verts.push_back(vec3(i * b->second[0],
                                         j * b->second[1],
                                         k * b->second[2]));
                }
            }
        }
        nodes[b->first] = new convex_node(b->first, verts);
    }

    transform3 base = ms->get_base_xform();
    double occlusion_time = 0;

    // Iterate through waypoints
    std::vector<std::vector<double> >::iterator w = t.waypoints.begin();
    w++;
    int w_i = 1; // Start at index 1 for time subtraction
    for (; w != t.waypoints.end(); w++) {
        std::map<std::string, double> joint_map;
        for (int i = 0; i < w->size(); i++) {
            joint_map[t.joints[i]] = (*w)[i];
        }

        // Need to get new transforms for each link box at each waypoint
        std::vector<const sgnode*> occluders;

        std::map<std::string, transform3> wp_xf = ms->get_link_transforms_at(joint_map);
        std::map<std::string, convex_node*>::iterator n = nodes.begin();
        for (; n != nodes.end(); n++) {
            transform3 world = base * wp_xf[n->first];
            vec3 p, r, s;
            world.position(p);
            world.rotation(r);
            world.scale(s);
            n->second->set_trans(p, r, s);
            occluders.push_back(n->second);
        }

        double co =  convex_occlusion(views, occluders);
        if (co >= 0.25) { // 2 out of 8 box corners
             occlusion_time += (t.times[w_i] - t.times[w_i-1]);
        }
        w_i++;
    }

    std::map<std::string, convex_node*>::iterator d = nodes.begin();
    for (; d != nodes.end(); d++) {
        delete d->second;
    }

    //std::cout << "Occluded for " << occlusion_time << " out of "
    //          << t.times[t.length - 1] << std::endl;
    return occlusion_time;
}

objective* make_occlusion_time_objective(Symbol* cmd_rt,
                                            soar_interface* si,
                                            motor_state* ms,
                                            objective_input* oi) {
    return new occlusion_time_objective(cmd_rt, si, ms, oi);
}

objective_table_entry* occlusion_time_objective_entry() {
    objective_table_entry* e = new objective_table_entry();
    e->name = "occlusion-time";
    e->description = "Time that an object is occluded across trajectory";
    e->parameters["set-id"] = "Trajectory set";
    e->parameters["obstacle"] = "Object of interest";
    e->create = &make_occlusion_time_objective;
    return e;
}

#endif
