#ifndef SVS_H
#define SVS_H

#include <memory>
#include <vector>
#include <map>
#include <set>
#include "soar_interface.h"
#include "sgnode.h"
#include "common.h"
#include "svs_interface.h"
#include "cliproxy.h"

#ifdef ENABLE_ROS
#include <mutex>
#include "ros_interface.h"
#endif

class command;
class scene;
class drawer;
class image_descriptor;

#ifdef ENABLE_ROS
class pcl_image;
#else
class basic_image;
#endif
class motor;
class motor_state;
class motor_link;
class model_database;

/* working memory scene graph object - mediates between wmes and scene graph nodes */
class sgwme : public sgnode_listener
{
    public:
        sgwme(soar_interface* si, Symbol* ident, sgwme* parent, sgnode* node);
        ~sgwme();
        void node_update(sgnode* n, sgnode::change_type t, const std::string& update_info);
        Symbol* get_id()
        {
            return id;
        }
        sgnode* get_node()
        {
            return node;
        }
        std::map<sgwme*, wme*>* get_childs()
        {
            return &childs;
        }

    private:
        void add_child(sgnode* c);
        
        // Functions dealing with maintaining tags on sgnodes
        void update_tag(const std::string& tag_name);
        void delete_tag(const std::string& tag_name);
        void set_tag(const std::string& tag_name, const std::string& tag_value);
        
        sgwme*          parent;
        sgnode*         node;
        Symbol*         id;
        wme*            id_wme;
        soar_interface* soarint;
        
        std::map<sgwme*, wme*> childs;
        
        std::map<std::string, wme*> tags;
};


class svs;

struct command_entry
{
    std::string id;
    command* cmd;
    wme* cmd_wme;
    command_entry(std::string id, command* cmd, wme* cmd_wme): id(id), cmd(cmd), cmd_wme(cmd_wme) {}
    bool operator < (const command_entry e) const
    {
        return id.compare(e.id) < 0;
    }
};
typedef std::set<command_entry> command_set;
typedef command_set::iterator command_set_it;

std::string change_cmd(std::string name, transform3 xform);
std::string add_cmd(std::string name, transform3 xform);
std::string del_cmd(std::string name);

/*
 Each state in the state stack has its own SVS link, scene, etc.
*/
class svs_state : public cliproxy
{
    public:
        svs_state(svs* svsp, Symbol* state, soar_interface* soar, scene* scn);
        svs_state(Symbol* state, svs_state* parent);
        
        ~svs_state();
        
        void           process_cmds();
        void           update_cmd_results(int command_type);
        void           update_scene_num();
        void           clear_scene();
        void           sync_scene_robot();
        
        std::string    get_name() const
        {
            return name;
        }
        int            get_level() const
        {
            return level;
        }
        int            get_scene_num() const
        {
            return scene_num;
        }
        scene*         get_scene() const
        {
            return scn;
        }
        motor_state* get_motor_state() const
        {
            return ms;
        }
#ifdef ENABLE_ROS
        pcl_image*     get_image() const
#else
        basic_image*   get_image() const
#endif
        {
            return img;
        }
        Symbol*        get_state()
        {
            return state;
        }
        svs*           get_svs()
        {
            return svsp;
        }
        
        /*
         Should only be called by svs::state_deletion_callback to save top-state scene
         during init.
        */
        void disown_scene();
        
    private:
        void init();
        void collect_cmds(Symbol* id, std::set<wme*>& all_cmds);
        
        void proxy_get_children(std::map<std::string, cliproxy*>& c);
        void cli_out(const std::vector<std::string>& args, std::ostream& os);

        std::string     name;
        svs*            svsp;
        int             level;
        svs_state*      parent;
        scene*          scn;
        sgwme*          root;
        image_descriptor* imwme;
        motor_link*     mowme;
        soar_interface* si;
#ifdef ENABLE_ROS
        pcl_image*      img;
#else
        basic_image*    img;
#endif
        motor_state*    ms;

        Symbol* state;
        Symbol* svs_link;
        Symbol* scene_link;
        Symbol* img_link;
        Symbol* mtr_link;
        Symbol* cmd_link;
        
        int scene_num;
        wme* scene_num_wme;
        
        /* command changes per decision cycle */
        command_set curr_cmds;
};


class svs : public svs_interface, public cliproxy
{
    public:
        svs(agent* a);
        ~svs();
        
        void state_creation_callback(Symbol* goal);
        void state_deletion_callback(Symbol* goal);
        void output_callback();
        void input_callback();
        void add_sgel_input(const std::string& in);
        void add_joint_input(std::map<std::string, double>& in);
        void add_loc_input(transform3& in);
        
        std::string svs_query(const std::string& query);

#ifdef ENABLE_ROS
        void image_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& new_img);
#endif

        soar_interface* get_soar_interface()
        {
            return si;
        }
#ifdef ENABLE_ROS
       ros_interface* get_ros_interface() { return ri; }

        std::shared_ptr<motor> get_motor()
        {
            return mtr;
        }

        std::shared_ptr<model_database> get_model_db()
        {
            return models;
        }

        
#endif

        drawer* get_drawer() const
        {
            return draw;
        }
        
        bool do_cli_command(const std::vector<std::string>& args, std::string& output);
        
        bool is_enabled()
        {
            return enabled;
        }
        void set_enabled(bool is_enabled)
        {
            enabled = is_enabled;
        }
        
        std::string get_output() const
        {
            return "";
        }

        // dirty bit is true only if there has been a new command
        //   from soar or from SendSVSInput 
        //   (no need to recheck filters)
        static void mark_filter_dirty_bit()
        {
            svs::filter_dirty_bit = true;
        }

        static bool get_filter_dirty_bit()
        {
            return svs::filter_dirty_bit;
        }
        
    private:
        void proc_input(svs_state* s);
        
        void proxy_get_children(std::map<std::string, cliproxy*>& c);
        void cli_connect_viewer(const std::vector<std::string>& args, std::ostream& os);
        void cli_disconnect_viewer(const std::vector<std::string>& args, std::ostream& os);

#ifdef ENABLE_ROS
        ros_interface*            ri;
        std::mutex                sgel_in_mtx;
        std::mutex                joint_in_mtx;
        std::mutex                loc_in_mtx;
#endif
        std::shared_ptr<motor>    mtr;
        std::shared_ptr<model_database> models;

        soar_interface*           si;
        std::vector<svs_state*>   state_stack;
        std::vector<std::string>  env_inputs;
        std::map<std::string, double> joint_inputs;
        transform3                loc_input;
        std::string               env_output;
        mutable drawer*           draw;
        scene*                    scn_cache;      // temporarily holds top-state scene during init
        
        bool enabled;

        static bool filter_dirty_bit;
};

#endif
