#include "model_database.h"

#include <iostream>
#include <fstream>

void model_database::reload() {
  collision_models.clear();
  grasps.clear();
  init();
}

bool model_database::is_in_database(std::string id) {
  return (db_has_grasps(id) || db_has_model(id));
}

bool model_database::db_has_grasps(std::string id) {
    if (id == "") return false;

    for (std::map<std::string, std::vector<grasp_pair> >::iterator j =
             grasps.begin(); j != grasps.end(); j++) {
        if (j->first.find(id) != std::string::npos ||
            id.find(j->first) != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool model_database::db_has_model(std::string id) {
    if (id == "") return false;

    for (std::map<std::string, std::vector<sub_shape> >::iterator p =
             collision_models.begin(); p != collision_models.end(); p++) {
        if (p->first.find(id) != std::string::npos ||
            id.find(p->first) != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool model_database::model_is_complex(std::string id) {
    if (!db_has_model(id)) return false;

    if (collision_models[id].size() == 1) return false;
    else return true;
}

std::string model_database::find_db_name(std::string id) {
  for (std::map<std::string, std::vector<grasp_pair> >::iterator j =
         grasps.begin(); j != grasps.end(); j++) {
    if (j->first.find(id) != std::string::npos ||
        id.find(j->first) != std::string::npos) {
      return j->first;
    }
  }

  for (std::map<std::string, std::vector<sub_shape> >::iterator p =
         collision_models.begin(); p != collision_models.end(); p++) {
    if (p->first.find(id) != std::string::npos ||
        id.find(p->first) != std::string::npos) {
      return p->first;
    }
  }

  //std::cout << "Could not find database entry for " << id << std::endl;
  return "";
}

std::vector<sub_shape> model_database::get_model(std::string db_name) {
    if (!db_has_model(db_name)) return std::vector<sub_shape>();
    return collision_models[db_name];
}

int model_database::num_grasps(std::string db_name) {
    if (!db_has_grasps(db_name)) return 0;
    return grasps[db_name].size();
}

std::vector<grasp_pair> model_database::all_grasps(std::string db_name) {
    if (!db_has_grasps(db_name)) return std::vector<grasp_pair>();
    return grasps[db_name];
}

grasp_pair model_database::grasp_at_index(std::string db_name, int index) {
    if (!db_has_grasps(db_name)) return grasp_pair();
    return grasps[db_name][index];
}

// Reads in json specifying data about the objects the robot may find
void model_database::init() {
  // OPEN FILE
  std::ifstream jsonfile("/home/ecat/Soar/Core/SVS/object_info.json");
  int length = 0;
  if (jsonfile) {
    jsonfile.seekg(0, jsonfile.end);
    length = jsonfile.tellg();
    jsonfile.seekg(0, jsonfile.beg);
  }
  else {
      std::cout << "Could not init object database at all!" << std::endl;
    return;
  }

  // READ FILE
  char* buf = new char[length];
  jsonfile.read(buf, length);
  // Make sure there is nothing else after the json object
  for (int i = length-1; i > 0; i--) {
    if (buf[i] != '}') buf[i] = 0;
    else break;
  }

  // CONVERT TO JSON
  rapidjson::Document d;
  d.Parse(buf);
  delete[] buf;
  jsonfile.close();

  if (d.HasParseError()) {
      std::cout << "Failed to parse json file with error code "
                << d.GetParseError() << " at "
                << d.GetErrorOffset() << std::endl;
  }

  // CHECK THAT THERE IS A LIST OF OBJECTS
  if (d.HasMember("objects")) {
      std::cout << "model_database successfully read object database json file!"
                << std::endl;
  }

  const rapidjson::Value& objs = d["objects"];
  assert(objs.IsArray());

  std::cout << "There are " << objs.Size()
            << " objects in the object_info file." << std::endl;

  // GO THROUGH ALL THE OBJECTS
  for (int i = 0; i < objs.Size(); i++) {
      if(!objs[i].HasMember("shapes") || !objs[i]["shapes"].IsArray()) {
          std::cout << "Database object " << objs[i]["name"].GetString()
                    << " has no shapes, will not be added." << std::endl;
          continue;
      }

      std::vector<sub_shape> shapeVec;
      bool shape_err = false;
      std::string err_str = "";
      for (int j = 0; j < objs[i]["shapes"].Size(); j++) {
          obstacle shape;
          if (objs[i]["shapes"][j]["shape"] == "box") {
              shape.geometry = BOX_OBSTACLE;
          } else if (objs[i]["shapes"][j]["shape"] == "sphere") {
              shape.geometry = BALL_OBSTACLE;
          } else if (objs[i]["shapes"][j]["shape"] == "convex") {
              shape.geometry = CONVEX_OBSTACLE;
          } else {
              shape_err = true;
              err_str = "Unknown object type.";
              break;
          }

          if (shape.geometry == BOX_OBSTACLE) {
              if (objs[i]["shapes"][j]["dimensions"].Size() != 3) {
                  shape_err = true;
                  err_str = "Boxes need three elements in their dimensions.";
                  break;
              }
              shape.box_dim[0] = objs[i]["shapes"][j]["dimensions"][0].GetDouble();
              shape.box_dim[1] = objs[i]["shapes"][j]["dimensions"][1].GetDouble();
              shape.box_dim[2] = objs[i]["shapes"][j]["dimensions"][2].GetDouble();
          } else if (shape.geometry == BALL_OBSTACLE) {
              if (objs[i]["shapes"][j]["dimensions"].Size() != 1) {
                  shape_err = true;
                  err_str = "Spheres need one element in their dimensions.";
                  break;
              }
              shape.ball_radius = objs[i]["shapes"][j]["dimensions"][0].GetDouble();
          } else if (shape.geometry == CONVEX_OBSTACLE) {
              if ((objs[i]["shapes"][j]["dimensions"].Size() % 3) != 0) {
                  shape_err = true;
                  err_str = "Convex shapes need three values for each point.";
                  break;
              }

              for (int k = 0; k < objs[i]["shapes"][j]["dimensions"].Size(); k += 3) {
                  vec3 pt;
                  pt[0] = objs[i]["shapes"][j]["dimensions"][k].GetDouble();
                  pt[1] = objs[i]["shapes"][j]["dimensions"][k+1].GetDouble();
                  pt[2] = objs[i]["shapes"][j]["dimensions"][k+2].GetDouble();
                  shape.convex_pts.push_back(pt);
              }
          }

          transform3 xform;
          if (!objs[i]["shapes"][j].HasMember("transform")) {
              xform = transform3::identity();
          } else {
              if (!objs[i]["shapes"][j]["transform"].HasMember("translation") ||
                  !objs[i]["shapes"][j]["transform"]["translation"].IsArray() ||
                  !objs[i]["shapes"][j]["transform"].HasMember("rotation") ||
                  !objs[i]["shapes"][j]["transform"]["rotation"].IsArray()) {
                  shape_err = true;
                  err_str = "Shape transform info is not correct.";
                  break;
              }
              vec3 trans(objs[i]["shapes"][j]["transform"]["translation"][0].GetDouble(),
                         objs[i]["shapes"][j]["transform"]["translation"][1].GetDouble(),
                         objs[i]["shapes"][j]["transform"]["translation"][2].GetDouble());

              vec4 rot(objs[i]["shapes"][j]["transform"]["rotation"][0].GetDouble(),
                       objs[i]["shapes"][j]["transform"]["rotation"][1].GetDouble(),
                       objs[i]["shapes"][j]["transform"]["rotation"][2].GetDouble(),
                       objs[i]["shapes"][j]["transform"]["rotation"][3].GetDouble());

              xform = transform3(trans, rot);
          }

          sub_shape ss = std::make_pair(xform, shape);
          shapeVec.push_back(ss);
      }

      if (shape_err) {
          std::cout << "Error importing shapes for " << objs[i]["name"].GetString() << ": "
                    << err_str << std::endl;
          continue;
      }
      collision_models.insert(std::pair<std::string,
                              std::vector<sub_shape> >(objs[i]["name"].GetString(),
                                                       shapeVec));

    if(!objs[i].HasMember("grasps") || !objs[i]["grasps"].IsArray()) {
        //std::cout << "Database object " << objs[i]["name"].GetString()
        //          <<" has no grasp information, only collision model." << std::endl;
        continue;
    }

    std::vector<grasp_pair> allGrasps;
    for (int j = 0; j < objs[i]["grasps"].Size(); j++) {
      if (objs[i]["grasps"][j]["first"].Size() != 6 ||
          objs[i]["grasps"][j]["second"].Size() != 6) {
          std::cout << "Wrong number of elements in grasp xyzrpy for object "
                    << objs[i]["name"].GetString() << ", grasp " << j << std::endl;
          continue;
      }
      vec3 rpy1(objs[i]["grasps"][j]["first"][3].GetDouble(),
               objs[i]["grasps"][j]["first"][4].GetDouble(),
               objs[i]["grasps"][j]["first"][5].GetDouble());
      vec3 xyz1(objs[i]["grasps"][j]["first"][0].GetDouble(),
               objs[i]["grasps"][j]["first"][1].GetDouble(),
               objs[i]["grasps"][j]["first"][2].GetDouble());
      transform3 t1(xyz1, rpy1, vec3(1, 1, 1));

      vec3 rpy2(objs[i]["grasps"][j]["second"][3].GetDouble(),
                  objs[i]["grasps"][j]["second"][4].GetDouble(),
                  objs[i]["grasps"][j]["second"][5].GetDouble());
      vec3 xyz2(objs[i]["grasps"][j]["second"][0].GetDouble(),
                objs[i]["grasps"][j]["second"][1].GetDouble(),
                objs[i]["grasps"][j]["second"][2].GetDouble());
      transform3 t2(xyz2, rpy2, vec3(1, 1, 1));

      grasp_pair gp = std::make_pair(t1, t2);
      allGrasps.push_back(gp);
    }
    grasps.insert(std::pair<std::string, std::vector<grasp_pair> >(objs[i]["name"].GetString(),
                                                                  allGrasps));
  }

  std::cout << "model_database loaded collision info for " << collision_models.size()
            << " objects and grasp info for " << grasps.size()
            << " objects" << std::endl;
}
