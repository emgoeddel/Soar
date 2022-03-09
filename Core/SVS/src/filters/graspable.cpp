/***************************************************
 *
 * File: filters/graspable.cpp
 *
 * Graspable Filters
 *
 * Filter graspable : map_filter<bool>
 * 	 Parameters:
 * 	 	sgnode a
 * 	 Returns:
 *              True if a is graspable (corresponds to an object with grasp poses in the db)
 *              False if a is not in the db or has no grasp poses
 *
 * Filter graspable_select : select_filter<sgnode*>
 * 	Parameters:
 * 		sgnode a
 * 	Returns:
 *              sgnode a if a is graspable (definition above)
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

////// filter graspable //////
class graspable_filter : public map_filter<bool> {
public:
	graspable_filter(Symbol *root,
                         soar_interface *si,
                         scene *scn,
                         filter_input *input) : map_filter<bool>(root, si, input),
                                                scn(scn)
	{}

	bool compute(const filter_params *p, bool& out) {
		sgnode *a;

		if (!get_filter_param(this, p, "a", a)) {
			set_status("expecting parameter a");
			return false;
		}

                out = scn->node_has_grasp(a->get_id());
                return true;
	}

private:
	scene *scn;
};

filter* make_graspable_filter(Symbol* root, soar_interface* si, scene* scn, filter_input* input)
{
    return new graspable_filter(root, si, scn, input);
}

filter_table_entry* graspable_filter_entry()
{
    filter_table_entry* e = new filter_table_entry();
    e->name = "graspable";
    e->description = "Returns true if a has grasp poses";
    e->parameters["a"] = "Sgnode a";
    e->create = &make_graspable_filter;
    return e;
}

////// filter graspable_select //////

class graspable_select_filter : public select_filter<sgnode*> {
public:
    graspable_select_filter(Symbol *root,
                            soar_interface *si,
                            scene *scn,
                            filter_input *input) : select_filter<sgnode*>(root,
                                                                          si,
                                                                          input),
                                                   scn(scn)
	{}

    bool compute(const filter_params *p, sgnode*& out, bool& select) {
		sgnode *a;

		if (!get_filter_param(this, p, "a", a)) {
			set_status("expecting parameter a");
			return false;
		}

                out = a;
                if (scn->node_has_grasp(a->get_id())) {
                    select = true;
                } else {
                    select = false;
                }
                return true;
	}

private:
	scene *scn;
};

filter* make_graspable_select_filter(Symbol* root, soar_interface* si, scene* scn, filter_input* input)
{
    return new graspable_select_filter(root, si, scn, input);
}

filter_table_entry* graspable_select_filter_entry()
{
    filter_table_entry* e = new filter_table_entry();
    e->name = "graspable_select";
    e->description = "Select a if it has grasp poses";
    e->parameters["a"] = "Sgnode a";
    e->create = &make_graspable_select_filter;
    return e;
}
