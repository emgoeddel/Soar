#ifndef MODEL_DATABASE_H
#define MODEL_DATABASE_H

#include "mat.h"
#include "motor_types.h"
#include "rapidjson/document.h"

typedef std::pair<transform3, obstacle> sub_shape;
typedef std::pair<transform3, transform3> grasp_pair;

/*
 * model_database class
 *
 * Reads in information about the objects that may come in from
 * Gazebo so that we can connect the names to actual geometries.
 *
 */

class model_database {
public:
    model_database() { init(); }

    void reload();

    bool is_in_database(std::string id);
    bool db_has_grasps(std::string id);
    bool db_has_model(std::string id);
    std::string find_db_name(std::string id);

    std::vector<sub_shape> get_model(std::string db_name);
    int num_grasps(std::string db_name);
    std::vector<grasp_pair> all_grasps(std::string db_name);
    grasp_pair grasp_at_index(std::string db_name, int index);

private:
    void init();

    std::map<std::string, std::vector<sub_shape> > collision_models;
    std::map<std::string, std::vector<grasp_pair> > grasps;
};

#endif
