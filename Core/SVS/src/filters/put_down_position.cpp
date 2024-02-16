/***************************************************
 *
 * File: filters/put_down_position.cpp
 *
 * Put Down Position Calculation Filters
 *
 * Filter put_down_area : map_filter<vec6>
 * 	 Parameters:
 * 	 	sgnode holding
 *              sgnode target
 *              frame < world self object >
 * 	 Returns:
 *              A box target that will place <holding> above <target> for a put-down
 *              NOTE BIG HACK: Returns a vec6 where x, y, z are the center of an AABB
 *                             and roll, pitch, and yaw are actually its DIMENSIONS
 *
 * NOTE ALSO that most of this assumes top-down grasps and axis-aligned target
 *           object; generalizing is definitely possible but requires more thought
 *           and also more informaiton than a vec6 to convey the result
 *********************************************************/
#include <iostream>
#include <assert.h>
#include <string>
#include <map>
#include "filter.h"
#include "sgnode.h"
#include "scene.h"
#include "filter_table.h"

////// filter put_down_area //////
class put_down_area_filter : public map_filter<vec6> {
public:
    put_down_area_filter(Symbol *root,
                         soar_interface *si,
                         scene *scn,
                         filter_input *input) :
        map_filter<vec6>(root, si, input),
        scn(scn) {}

    bool compute(const filter_params *p, vec6& out) {
        // Find held object information
        sgnode* h;

        if (!get_filter_param(this, p, "holding", h)) {
            set_status("expecting parameter holding");
            return false;
        }

        // Use held object grasp info to calculate position
        std::vector<std::pair<transform3, transform3> > grasp_pairs =
            scn->node_grasps(h->get_id());

        if (grasp_pairs.empty()) {
            set_status("no grasp information found for held object");
            return false;
        }

        box_node* h_box = static_cast<box_node*>(h);
        if (!h_box) {
            set_status("holding is not a box");
            return false;
        }
        vec3 h_dim = h_box->get_dimensions();


        // Find target information
        sgnode* t;
        if (!get_filter_param(this, p, "target", t)) {
            set_status("expecting parameter target");
            return false;
        }
        box_node* t_box = static_cast<box_node*>(t);
        if (!t_box) {
            set_status("target is not a box");
            return false;
        }
        vec3 t_dim = t_box->get_dimensions();

        std::string f;
        if (!get_filter_param(this, p, "frame", f)) {
            set_status("expecting frame parameter");
            return false;
        }

        vec6 pos;

        if (f == "object") {
            grasp_pairs[0].first.xyzrpy(pos);
        } else if (f == "world") {
            transform3 wt = t->get_world_trans()*grasp_pairs[0].first;
            wt.xyzrpy(pos);
        } else if (f == "self") {
            transform3 b = scn->get_self_root()->get_world_trans();
            transform3 o = t->get_world_trans()*grasp_pairs[0].first;

            transform3 st = b.inv()*o;
            st.xyzrpy(pos);
        } else {
            set_status("frame parameter invalid");
            return false;
        }

        // Assuming that the frame of the object is at its bottom, center (?)
        // And include some extra space to avoid mashing
        pos[2] += t_dim[2] + 0.02;

         // Swap in the target dimensions instead of orientation
        // We want an area the size of the target minus held object in x, y,
        // with a little fudge to avoid the very edge
        // but not much z variation
        pos[3] = t_dim[0] - h_dim[0] - 0.02;
        pos[4] = t_dim[1] - h_dim[1] - 0.02;
        pos[5] = 0.005;

        out = pos;
        return true;
    }

private:
    scene *scn;
};

filter* make_put_down_area_filter(Symbol* root, soar_interface* si, scene* scn, filter_input* input)
{
    return new put_down_area_filter(root, si, scn, input);
}

filter_table_entry* put_down_area_filter_entry()
{
    filter_table_entry* e = new filter_table_entry();
    e->name = "put_down_area";
    e->description = "Returns a box target where holding can be placed on target";
    e->parameters["holding"] = "Sgnode of held object";
    e->parameters["target"] = "Sgnode of target object";
    e->parameters["frame"] = "<world self object>";
    e->create = &make_put_down_area_filter;
    return e;
}
