/***************************************************
 *
 * File: filters/calculate_push.cpp
 *
 * Calculate Push Filter
 * 	Given two nodes, find a vector for the first to be pushed onto to the second
 * 	taking bounding boxes into consideration
 *  Assumes target location is axis-aligned, but object being pushed can be
 *  rotated
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
  bbox bbb = b->get_bounds();
  vec3 bb_min, bb_max;
  bbb.get_vals(bb_min, bb_max);
  vec3 cor3(bb_min[0], bb_max[1], 0);
  vec3 cor4(bb_max[0], bb_min[1], 0);

  transform3 obj_to_world = transform3(a->get_trans('p'), a->get_trans('r')).inverse();

  vec3 bb_min_rel = obj_to_world(bb_min);
  vec3 bb_max_rel = obj_to_world(bb_max);
  vec3 corner3 = obj_to_world(cor3);
  vec3 corner4 = obj_to_world(cor4);
  cout << "Location min: " << bb_min_rel[0] << ", " << bb_min_rel[1] << endl;
  cout << "Location max: " << bb_max_rel[0] << ", " << bb_max_rel[1] << endl;

  vector<vector<float> > lines;
  // min - 3
  vector<float> line1;
  line1.push_back((corner3[1] - bb_min_rel[1]) / (corner3[0] - bb_min_rel[0]));
  cout << "Line 1 slope: " << line1[0] << endl;
  line1.push_back(bb_min_rel[0]);
  line1.push_back(bb_min_rel[1]);
  line1.push_back(corner3[0]);
  line1.push_back(corner3[1]);
  lines.push_back(line1);
  // min - 4
  vector<float> line2;
  line2.push_back((corner4[1] - bb_min_rel[1]) / (corner4[0] - bb_min_rel[0]));
  cout << "Line 2 slope: " << line2[0] << endl;
  line2.push_back(bb_min_rel[0]);
  line2.push_back(bb_min_rel[1]);
  line2.push_back(corner4[0]);
  line2.push_back(corner4[1]);
  lines.push_back(line2);
  // max - 3
  vector<float> line3;
  line3.push_back((corner3[1] - bb_max_rel[1]) / (corner3[0] - bb_max_rel[0]));
  cout << "Line 3 slope: " << line3[0] << endl;
  line3.push_back(bb_max_rel[0]);
  line3.push_back(bb_max_rel[1]);
  line3.push_back(corner3[0]);
  line3.push_back(corner3[1]);
  lines.push_back(line3);
  // max - 4
  vector<float> line4;
  line4.push_back((corner4[1] - bb_max_rel[1]) / (corner4[0] - bb_max_rel[0]));
  cout << "Line 4 slope: " << line4[0] << endl;
  line4.push_back(bb_max_rel[0]);
  line4.push_back(bb_max_rel[1]);
  line4.push_back(corner4[0]);
  line4.push_back(corner4[1]);
  lines.push_back(line4);

  // Figure out if already on

  float x_push = 100;
  float y_push = 100;
  for (int i = 0; i < 4; i++) {
    float line_x_int = lines[i][2] - lines[i][0]*lines[i][1];
    float line_y_int = -(lines[i][2]/lines[i][0]) + lines[i][1];
    if ((line_x_int < lines[i][2] && line_x_int > lines[i][4]) ||
        (line_x_int > lines[i][2] && line_x_int < lines[i][4]))
      {
        cout << "Valid x intercept found: " << line_x_int << endl;
        if (line_x_int - pa[1] < y_push)
          y_push = line_x_int - pa[1];
      }
    if ((line_y_int < lines[i][1] && line_y_int > lines[i][3]) ||
        (line_y_int > lines[i][1] && line_y_int < lines[i][3]))
      {
        cout << "Valid y intercept found: " << line_y_int << endl;
        if (line_y_int - pa[0] < x_push)
          x_push = line_y_int - pa[0];
      }
  }

  // No push available
  if (x_push == 100 && y_push == 100) {
    vec3 push(0, 0, -1);
    return push;
  }

  if (x_push == 100) {
    vec3 ypush(0, y_push, 0);
    return ypush;
  }

  if (y_push == 100) {
    vec3 xpush(x_push, 0, 0);
    return xpush;
  }

  if (fabs(y_push) < fabs(x_push)) {
    vec3 ypush(0, y_push, 0);
    return ypush;
  }
  else {
    vec3 xpush(x_push, 0, 0);
    return xpush;
  }
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

