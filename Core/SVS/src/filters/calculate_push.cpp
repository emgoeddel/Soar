/***************************************************
 *
 * File: filters/calculate_push.cpp
 *
 * Calculate Placement Filter
 * 	given two nodes, find a vector for the first to be pushed onto to the second
 * 	taking bounding boxes into consideration
 *
 * Filter calculate_placement : map_filter<vec3>
 *   Parameters:
 *    sgnode a - the node being placed
 *    sgnode b - the reference node
 *   Returns:
 *    vec3 - a direction from the location of node a such that it can move along
 *           one of its axes to end up on node b
 *           z direction will always be 0; either x or y but not both will be filled
 *
 *********************************************************/
#include "sgnode_algs.h"
#include "filters/base_node_filters.h"
#include "scene.h"
#include "filter_table.h"

#include <string>

using namespace std;

vec3 calculate_push(const sgnode* a, const sgnode* b){
  vec3 pa = a->get_centroid();
  vec3 pb = b->get_centroid();

  vec3 axes[3];
  axes[0] = vec3(1.0, 0.0, 0.0);
  axes[1] = vec3(0.0, 1.0, 0.0);
  axes[2] = vec3(0.0, 0.0, 1.0);

  // Transform axes relative to the object to be pushed
  transform3 rot('r', a->get_trans('r'));
  for(int dim = 0; dim < 3; dim++)
    {
      axes[dim] = rot(axes[dim]);
    }

  vec3 pos(0.1, 0, 0);
  return pos;
}

class calculate_push_filter : public map_filter<vec3>
{
public:
  calculate_push_filter(Symbol* root, soar_interface* si, filter_input* input)
    : map_filter<vec3>(root, si, input)
  {}
  bool compute(const filter_params* params, vec3& out)
  {
    sgnode* a;
    if (!get_filter_param(this, params, "a", a))
      {
        set_status("expecting sgnode parameter 'a'");
        return false;
      }

    sgnode* b;
    if (!get_filter_param(this, params, "b", b))
      {
        set_status("expecting sgnode parameter 'b'");
        return false;
      }

    out = calculate_push(a, b);
    return true;
  }
};

///// filter calculate_push //////
filter* make_calculate_push_filter(Symbol* root, soar_interface* si, scene* scn, filter_input* input)
{
  return new calculate_push_filter(root, si, input);
}

filter_table_entry* calculate_push_filter_entry()
{
  filter_table_entry* e = new filter_table_entry();
  e->name = "calculate_push";
  e->description = "Outputs a vector to push node a onto node b";
  e->parameters["a"] = "The node being pushed";
  e->parameters["b"] = "The node acting as a reference to place node a";
  e->create = &make_calculate_push_filter;
  return e;
}

