/***************************************************
 *
 * File: filters/grasp_positions.cpp
 *
 * Grasp Position Access Filters
 *
 * Filter grasp_first : map_filter<vec6>
 * 	 Parameters:
 * 	 	sgnode a
 *              frame < world self object >
 * 	 Returns:
 *              A single grasp approach pose for a (or error if a is not graspable)
 *
 * Filter grasp_second : map_filter<vec6>
 * 	 Parameters:
 * 	 	sgnode a
 *              frame < world self object >
 * 	 Returns:
 *              A single grasp final pose for a (or error if a is not graspable)
 *
 *********************************************************/
#include <iostream>
#include <assert.h>
#include <string>
#include <map>
#include "filter.h"
#include "sgnode.h"
#include "scene.h"
#include "filter_table.h"

////// filter grasp_first //////
class grasp_first_filter : public map_filter<vec6> {
public:
    grasp_first_filter(Symbol *root,
                       soar_interface *si,
                       scene *scn,
                       filter_input *input) :
        map_filter<vec6>(root, si, input),
        scn(scn) {}

    bool compute(const filter_params *p, vec6& out) {
        sgnode *a;

        if (!get_filter_param(this, p, "a", a)) {
            set_status("expecting parameter a");
            return false;
        }

        std::string f;
        if (!get_filter_param(this, p, "frame", f)) {
            set_status("expecting frame parameter");
            return false;
        }

        if (!scn->node_has_grasp(a->get_id())) {
            set_status("a is not graspable");
            return false;
        }

        std::vector<std::pair<transform3, transform3> > grasp_pairs = scn->node_grasps(a->get_id());
        if (grasp_pairs.empty()) {
            set_status("a is not graspable");
            return false;
        }

        vec6 pos;

        if (f == "object") {
            grasp_pairs[0].first.xyzrpy(pos);
        } else if (f == "world") {
            transform3 wt = a->get_world_trans()*grasp_pairs[0].first;
            wt.xyzrpy(pos);
        } else if (f == "self") {
            if (!scn->get_self_root()) {
                set_status("no self in scene");
                return false;
            }

            transform3 b = scn->get_self_root()->get_world_trans();
            transform3 o = a->get_world_trans()*grasp_pairs[0].first;

            transform3 st = b.inv()*o;
            st.xyzrpy(pos);
        } else {
            set_status("frame parameter invalid");
            return false;
        }

        out = pos;
        return true;
    }

private:
    scene *scn;
};

filter* make_grasp_first_filter(Symbol* root, soar_interface* si, scene* scn, filter_input* input)
{
    return new grasp_first_filter(root, si, scn, input);
}

filter_table_entry* grasp_first_filter_entry()
{
    filter_table_entry* e = new filter_table_entry();
    e->name = "grasp_first";
    e->description = "Returns a 6DOF grasp approach pose for a";
    e->parameters["a"] = "Sgnode a";
    e->parameters["frame"] = "<world self object>";
    e->create = &make_grasp_first_filter;
    return e;
}

////// filter grasp_second //////
class grasp_second_filter : public map_filter<vec6> {
public:
    grasp_second_filter(Symbol *root,
                       soar_interface *si,
                       scene *scn,
                       filter_input *input) :
        map_filter<vec6>(root, si, input),
        scn(scn) {}

    bool compute(const filter_params *p, vec6& out) {
        sgnode *a;

        if (!get_filter_param(this, p, "a", a)) {
            set_status("expecting parameter a");
            return false;
        }

        std::string f;
        if (!get_filter_param(this, p, "frame", f)) {
            set_status("expecting frame parameter");
            return false;
        }

        if (!scn->node_has_grasp(a->get_id())) {
            set_status("a is not graspable");
            return false;
        }

        std::vector<std::pair<transform3, transform3> > grasp_pairs = scn->node_grasps(a->get_id());
        if (grasp_pairs.empty()) {
            set_status("a is not graspable");
            return false;
        }

        vec6 pos;

        if (f == "object") {
            grasp_pairs[0].second.xyzrpy(pos);
        } else if (f == "world") {
            transform3 wt = a->get_world_trans()*grasp_pairs[0].second;
            wt.xyzrpy(pos);
        } else if (f == "self") {
            transform3 b = scn->get_self_root()->get_world_trans();
            transform3 o = a->get_world_trans()*grasp_pairs[0].second;

            transform3 st = b.inv()*o;
            st.xyzrpy(pos);
        } else {
            set_status("frame parameter invalid");
            return false;
        }

        out = pos;
        return true;
    }

private:
    scene *scn;
};

filter* make_grasp_second_filter(Symbol* root, soar_interface* si, scene* scn, filter_input* input)
{
    return new grasp_second_filter(root, si, scn, input);
}

filter_table_entry* grasp_second_filter_entry()
{
    filter_table_entry* e = new filter_table_entry();
    e->name = "grasp_second";
    e->description = "Returns a 6DOF grasping pose for a";
    e->parameters["a"] = "Sgnode a";
    e->parameters["frame"] = "<world self object>";
    e->create = &make_grasp_second_filter;
    return e;
}

////// filter grasp_vector //////
class grasp_vector_filter : public map_filter<vec3> {
public:
    grasp_vector_filter(Symbol *root,
                       soar_interface *si,
                       scene *scn,
                       filter_input *input) :
        map_filter<vec3>(root, si, input),
        scn(scn) {}

    bool compute(const filter_params *p, vec3& out) {
        sgnode *a;

        if (!get_filter_param(this, p, "a", a)) {
            set_status("expecting parameter a");
            return false;
        }
        if (!scn->node_has_grasp(a->get_id())) {
            set_status("a is not graspable");
            return false;
        }

        std::vector<std::pair<transform3, transform3> > grasp_pairs =
            scn->node_grasps(a->get_id());
        if (grasp_pairs.empty()) {
            set_status("a is not graspable");
            return false;
        }

        vec3 pos1, pos2;
        grasp_pairs[0].first.position(pos1);
        grasp_pairs[0].second.position(pos2);
        vec3 diff = pos2 - pos1;

        out = diff;
        return true;
    }

private:
    scene *scn;
};

filter* make_grasp_vector_filter(Symbol* root, soar_interface* si, scene* scn, filter_input* input)
{
    return new grasp_vector_filter(root, si, scn, input);
}

filter_table_entry* grasp_vector_filter_entry()
{
    filter_table_entry* e = new filter_table_entry();
    e->name = "grasp_vector";
    e->description = "Returns a vector from first to second grasp poses of a";
    e->parameters["a"] = "Sgnode a";
    e->create = &make_grasp_vector_filter;
    return e;
}
