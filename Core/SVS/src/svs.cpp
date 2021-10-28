#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <iterator>
#include <utility>
#include <algorithm>
#include <fstream>

#include "svs.h"
#include "command.h"
#include "sgnode.h"
#include "soar_interface.h"
#include "scene.h"
#include "image.h"
#include "common.h"
#include "filter_table.h"
#include "command_table.h"
#include "objective_table.h"
#include "drawer.h"

#include "symbol.h"

#include "motor/motor.h"
#include "motor_state.h"

using namespace std;

typedef map<string, command*>::iterator cmd_iter;

svs_interface* make_svs(agent* a)
{
    return new svs(a);
}


sgwme::sgwme(soar_interface* si, Symbol* ident, sgwme* parent, sgnode* node)
    : soarint(si), id(ident), parent(parent), node(node)
{
    node->listen(this);
    id_wme = soarint->make_wme(id, si->get_common_syms().id, node->get_id());

    if (node->is_group())
    {
        group_node* g = node->as_group();
        for (size_t i = 0; i < g->num_children(); ++i)
        {
            add_child(g->get_child(i));
        }
    }
    
    const tag_map& node_tags = node->get_all_tags();
    tag_map::const_iterator ti;
    for (ti = node_tags.begin(); ti != node_tags.end(); ti++)
    {
        set_tag(ti->first, ti->second);
    }
}

sgwme::~sgwme()
{
    map<sgwme*, wme*>::iterator i;
    
    if (node)
    {
        node->unlisten(this);
    }
    soarint->remove_wme(id_wme);
    
    map<string, wme*>::iterator ti;
    for (ti = tags.begin(); ti != tags.end(); ti++)
    {
        soarint->remove_wme(ti->second);
    }
    
    for (i = childs.begin(); i != childs.end(); ++i)
    {
        i->first->parent = NULL;
        delete i->first;
        soarint->remove_wme(i->second);
    }
    if (parent)
    {
        map<sgwme*, wme*>::iterator ci = parent->childs.find(this);
        assert(ci != parent->childs.end());
        soarint->remove_wme(ci->second);
        parent->childs.erase(ci);
    }
}

void sgwme::node_update(sgnode* n, sgnode::change_type t, const std::string& update_info)
{
    int added_child = 0;
    group_node* g;
    switch (t)
    {
        case sgnode::CHILD_ADDED:
            if (parse_int(update_info, added_child))
            {
                g = node->as_group();
                add_child(g->get_child(added_child));
            }
            break;
        case sgnode::DELETED:
            node = NULL;
            delete this;
            break;
        case sgnode::TAG_CHANGED:
            update_tag(update_info);
            break;
        case sgnode::TAG_DELETED:
            delete_tag(update_info);
            break;
        default:
            break;
    };
}

void sgwme::add_child(sgnode* c)
{
    char letter;
    string cid = c->get_id();
    sgwme* child;
    
    if (cid.size() == 0 || !isalpha(cid[0]))
    {
        letter = 'n';
    }
    else
    {
        letter = cid[0];
    }
    wme* cid_wme = soarint->make_id_wme(id, "child");
    
    child = new sgwme(soarint, soarint->get_wme_val(cid_wme), this, c);
    childs[child] = cid_wme;
}

void sgwme::set_tag(const string& tag_name, const string& tag_value)
{
    Symbol* rootID = id;
    std::string att = tag_name;
    
    wme* value_wme;
    if (map_get(tags, tag_name, value_wme))
    {
        soarint->remove_wme(value_wme);
    }
    tags[tag_name] = soarint->make_wme(rootID, att, tag_value);
}

void sgwme::update_tag(const string& tag_name)
{
    string tag_value;
    if (node->get_tag(tag_name, tag_value))
    {
        set_tag(tag_name, tag_value);
    }
}

void sgwme::delete_tag(const string& tag_name)
{
    wme* value_wme;
    if (map_get(tags, tag_name, value_wme))
    {
        soarint->remove_wme(value_wme);
        tags.erase(tag_name);
    }
}


svs_state::svs_state(svs* svsp, Symbol* state, soar_interface* si, scene* scn)
    : svsp(svsp), parent(NULL), state(state), si(si), level(0),
      scene_num(-1), scene_num_wme(NULL), scn(scn), img(NULL), ms(NULL),
      scene_link(NULL)
{
    assert(state->is_top_state());
    state->get_id_name(name);
    init();
}

svs_state::svs_state(Symbol* state, svs_state* parent)
    : parent(parent), state(state), svsp(parent->svsp), si(parent->si),
      level(parent->level + 1), scene_num(-1),
      scene_num_wme(NULL), scn(NULL), img(NULL), ms(NULL),
      scene_link(NULL)
{
    assert(state->get_parent_state() == parent->state);
    init();
}

svs_state::~svs_state()
{
    command_set_it i, iend;

    for (i = curr_cmds.begin(), iend = curr_cmds.end(); i != iend; ++i)
    {
        delete i->cmd;
    }

    if (scn)
    {
        svsp->get_drawer()->delete_scene(scn->get_name());
        delete scn; // results in root being deleted also
    }

    if (img) {
        delete imwme;
        delete img;
    }

    if (ms) {
        delete mowme;
        delete ms;
    }
}

void svs_state::init()
{
    common_syms& cs = si->get_common_syms();

    state->get_id_name(name);
    svs_link = si->get_wme_val(si->make_svs_wme(state));
    cmd_link = si->get_wme_val(si->make_id_wme(svs_link, cs.cmd));
    scene_link = si->get_wme_val(si->make_id_wme(svs_link, cs.scene));
    img_link = si->get_wme_val(si->make_id_wme(svs_link, cs.image));
    mtr_link = si->get_wme_val(si->make_id_wme(svs_link, cs.motor));

    if (!scn)
    {
        if (parent)
        {
            scn = parent->scn->clone(name);
        }
        else
        {
            // top state
            scn = new scene(name, svsp);
            scn->set_draw(true);
        }
    }

    scn->refresh_draw();
    root = new sgwme(si, scene_link, (sgwme*) NULL, scn->get_root());

    if (!img) {
#ifdef ENABLE_ROS
        img = new pcl_image();
#else
        img = new basic_image();
#endif
        if (parent) {
            img->copy_from(parent->img);
        }
        imwme = new image_descriptor(si, img_link, img);
    }

    if (!ms) {
        ms = new motor_state(svsp->get_motor(), scn, name);

        if (parent) {
            ms->copy_from(parent->ms);
        }
        mowme = new motor_link(si, mtr_link, ms);
    }
}

void svs_state::update_scene_num()
{
    long curr;
    if (scene_num_wme)
    {
        if (!get_symbol_value(si->get_wme_val(scene_num_wme), curr))
        {
            exit(1);
        }
        if (curr == scene_num)
        {
            return;
        }
        si->remove_wme(scene_num_wme);
    }
    if (scene_num >= 0)
    {
        scene_num_wme = si->make_wme(svs_link, "scene-num", scene_num);
    }
}

void svs_state::update_cmd_results(int command_type)
{
    command_set_it i;
    for (i = curr_cmds.begin(); i != curr_cmds.end(); ++i)
    {
        if (i->cmd->command_type() == command_type)
        {
            i->cmd->update();
        }
    }
}

void svs_state::process_cmds()
{
    wme_vector all;
    wme_vector::iterator all_it;
    si->get_child_wmes(cmd_link, all);
    
    command_set live_commands;
    for (all_it = all.begin(); all_it != all.end(); all_it++)
    {
        // Convert wme val to string
        Symbol* idSym = si->get_wme_val(*all_it);
        string cmdId;
        ;
        if (!idSym->get_id_name(cmdId))
        {
            // Not an identifier, continue;
            continue;
        }
        
        live_commands.insert(command_entry(cmdId, 0, *all_it));
    }
    // Do a diff on the curr_cmds list and the live_commands
    //   to find which have been added and which have been removed
    vector<command_set_it> old_commands, new_commands;
    command_set_it live_it = live_commands.begin();
    command_set_it curr_it = curr_cmds.begin();
    while (live_it != live_commands.end() || curr_it != curr_cmds.end())
    {
        if (live_it == live_commands.end())
        {
            old_commands.push_back(curr_it);
            curr_it++;
        }
        else if (curr_it == curr_cmds.end())
        {
            new_commands.push_back(live_it);
            live_it++;
        }
        else if (curr_it->id == live_it->id)
            // Find all commands removed from the svs command wme
        {
            curr_it++;
            live_it++;
        }
        else if (curr_it->id.compare(live_it->id) < 0)
        {
            old_commands.push_back(curr_it);
            curr_it++;
        }
        else
        {
            new_commands.push_back(live_it);
            live_it++;
        }
    }
    
    // Delete the command
    vector<command_set_it>::iterator old_it;
    for (old_it = old_commands.begin(); old_it != old_commands.end(); old_it++)
    {
        command_set_it old_cmd = *old_it;
        delete old_cmd->cmd;
        curr_cmds.erase(old_cmd);
    }
    
    // Add the new commands
    vector<command_set_it>::iterator new_it;
    for (new_it = new_commands.begin(); new_it != new_commands.end(); new_it++)
    {
        command_set_it new_cmd = *new_it;
        command* c = get_command_table().make_command(this, new_cmd->cmd_wme);
        if (c)
        {
            curr_cmds.insert(command_entry(new_cmd->id, c, 0));
            svs::mark_filter_dirty_bit();
        }
        else
        {
            string attr;
            get_symbol_value(si->get_wme_attr(new_cmd->cmd_wme), attr);
        }
    }
}

void svs_state::clear_scene()
{
    scn->clear();
}

std::string change_cmd(std::string name, transform3 xform) {
    vec3 p;
    xform.position(p);
    Eigen::Quaterniond q;
    xform.rotation(q);
    vec3 r = q.toRotationMatrix().eulerAngles(0, 1, 2);

    std::stringstream cmd;
    cmd << "change " << name;
    cmd << " p " << p.x() << " " << p.y() << " " << p.z();
    cmd << " r " << r.x() << " " << r.y() << " " << r.z();
    cmd << std::endl;
    return cmd.str();
}

std::string add_cmd(std::string name, std::string parent, transform3 xform) {
    vec3 p;
    xform.position(p);
    Eigen::Quaterniond q;
    xform.rotation(q);
    vec3 r = q.toRotationMatrix().eulerAngles(0, 1, 2);

    std::stringstream cmd;
    cmd << "add " << name << " " << parent;
    cmd << " p " << p.x() << " " << p.y() << " " << p.z();
    cmd << " r " << r.x() << " " << r.y() << " " << r.z();
    cmd << std::endl;
    return cmd.str();
}

std::string del_cmd(std::string name) {
    std::stringstream cmd;
    cmd << "delete " << name << std::endl;
    return cmd.str();
}

void svs_state::sync_scene_robot()
{
    // Don't worry about syncing if we don't have joint input
    if (!ms->has_joints()) return;
    // XXX Delete the robot from scene if so ^

    // Build up a vector of commands
    // XXX Maybe should do this directly since have access to scn?
    std::vector<std::string> cmds;
    // But only bother sending the update to SVS if something changed
    bool robot_changed = false;

    // Check if Fetch is in the scene, and if not just add it
    if (!scn->get_node(ms->robot_name())) {
        robot_changed = true;
        cmds.push_back(add_cmd(ms->robot_name(), "world", ms->get_base_xform()));
        std::map<std::string, transform3> links = ms->get_link_transforms();
        for (std::map<std::string, transform3>::iterator i = links.begin();
             i != links.end(); i++) {
            cmds.push_back(add_cmd(i->first, ms->robot_name(), i->second));
        }
    } else {
        // Check if the Fetch base has moved and update if so
        transform3 cur_xform = ms->get_base_xform();
        transform3 last_xform = scn->get_node(ms->robot_name())->get_world_trans();
        if (transform3::t_diff(cur_xform, last_xform)) {
            robot_changed = true;
            vec3 p;
            cur_xform.position(p);
            cmds.push_back(change_cmd(ms->robot_name(), cur_xform));
        }

        // Check if the links have moved and update if so
        // XXX Assumes robot's links don't change after it's first added!!
        std::map<std::string, transform3> links = ms->get_link_transforms();
        for (std::map<std::string, transform3>::iterator i = links.begin();
             i != links.end(); i++) {
            std::string n = i->first;

            transform3 cur_link = i->second;
            transform3 last_link = scn->get_node(n)->get_world_trans();

            if (transform3::t_diff(cur_link, last_link)) {
                robot_changed = true;
                cmds.push_back(change_cmd(n, cur_link));
            }
        }
    }

    if (robot_changed) {
        for (std::vector<std::string>::iterator i = cmds.begin();
             i != cmds.end(); i++)
        {
            scn->parse_sgel(*i);
        }
        svs::mark_filter_dirty_bit();
    }
}

void svs_state::proxy_get_children(map<string, cliproxy*>& c)
{
    c["scene"]        = scn;
}

void svs_state::disown_scene()
{
    delete root;
    scn = NULL;
}

svs::svs(agent* a)
    : scn_cache(NULL), enabled(false)
{
    si = new soar_interface(a);
    draw = new drawer();

#ifdef ENABLE_ROS
    ros_interface::init_ros();
    ri = new ros_interface(this);
    ri->start_ros();
#endif
    mtr = std::make_shared<motor>(ri->get_robot_desc());
}

bool svs::filter_dirty_bit = true;

svs::~svs()
{
    for (size_t i = 0, iend = state_stack.size(); i < iend; ++i)
    {
        delete state_stack[i];
    }
    if (scn_cache)
    {
        delete scn_cache;
    }

    get_command_table().del_entries();
    get_filter_table().del_entries();
    get_objective_table().del_entries();

    delete si;
    delete ri;
    delete draw;
}

void svs::state_creation_callback(Symbol* state)
{
    string type, msg;
    svs_state* s;
    
    if (state_stack.empty())
    {
        if (scn_cache)
        {
            scn_cache->verify_listeners();
        }
        s = new svs_state(this, state, si, scn_cache);
        scn_cache = NULL;
    }
    else
    {
        s = new svs_state(state, state_stack.back());
    }
    
    state_stack.push_back(s);
}

void svs::state_deletion_callback(Symbol* state)
{
    svs_state* s;
    s = state_stack.back();
    assert(state == s->get_state());
    if (state_stack.size() == 1)
    {
        // removing top state, save scene for reinit
        scn_cache = s->get_scene();
        s->disown_scene();
    }
    
    delete s;
    state_stack.pop_back();
}

void svs::proc_input(svs_state* s)
{
#ifdef ENABLE_ROS
    std::lock_guard<std::mutex> guard1(sgel_in_mtx);
#endif
    {
        std::lock_guard<std::mutex> guard2(loc_in_mtx);
        s->get_motor_state()->set_base_xform(loc_input);
    }
    {
        std::lock_guard<std::mutex> guard2(joint_in_mtx);
        s->get_motor_state()->set_joints(joint_inputs);
        if (s->get_motor_state()->get_joints_type() != "current") {
            std::cout << "Set joints to current!" << std::endl;
            s->get_motor_state()->set_joints_type("current");
        }
    }
    s->sync_scene_robot();

    for (size_t i = 0; i < env_inputs.size(); ++i)
    {
        strip(env_inputs[i], " \t");
        s->get_scene()->parse_sgel(env_inputs[i]);
    }
    if (env_inputs.size() > 0)
    {
        svs::mark_filter_dirty_bit();
    }
    env_inputs.clear();
}

void svs::output_callback()
{
    if (!enabled)
    {
        return;
    }
    vector<svs_state*>::iterator i;
    string sgel;
    
    for (i = state_stack.begin(); i != state_stack.end(); ++i)
    {
        (**i).process_cmds();
    }
    //for (i = state_stack.begin(); i != state_stack.end(); ++i)
    //{
    //    (**i).update_cmd_results(true);
    //}
    
}

void svs::input_callback()
{
    if (!enabled)
    {
        return;
    }
    svs_state* topstate = state_stack.front();
    proc_input(topstate);
    
    vector<svs_state*>::iterator i;
	
    for (i = state_stack.begin(); i != state_stack.end(); ++i)
    {
        (**i).update_cmd_results(SVS_WRITE_COMMAND);
    }

    for (i = state_stack.begin(); i != state_stack.end(); ++i)
    {
        (**i).update_cmd_results(SVS_READ_COMMAND);
    }

    svs::filter_dirty_bit = false;
}

#ifdef ENABLE_ROS
void svs::image_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& new_img)
{
    if (!enabled) return;
    if (!state_stack.front()->get_image()) return;

    // Updates only the top state image for now.
    state_stack.front()->get_image()->update_image(new_img);

    if (ri->get_image_source() != state_stack.front()->get_image()->get_source()) {
        state_stack.front()->get_image()->set_source(ri->get_image_source());
    }
}
#endif

/*
 This is a naive implementation. If this method is called concurrently
 with proc_input, the env_inputs vector will probably become
 inconsistent. This eventually needs to be replaced by a thread-safe FIFO.
 XXX: Lizzie used a mutex to lock the the env_inputs structure since
      ROS will update from a separate thread.
*/
void svs::add_sgel_input(const string& in)
{
#ifdef ENABLE_ROS
    std::lock_guard<std::mutex> guard(sgel_in_mtx);
#endif

    split(in, "\n", env_inputs);
}

void svs::add_joint_input(std::map<std::string, double>& in)
{
    std::lock_guard<std::mutex> guard(joint_in_mtx);

    joint_inputs = in;
}

void svs::add_loc_input(transform3& in)
{
    std::lock_guard<std::mutex> guard(loc_in_mtx);

    loc_input = in;
}

string svs::svs_query(const string& query)
{
    if (state_stack.size() == 0)
    {
        return "";
    }
    return state_stack[0]->get_scene()->parse_query(query);
}

void svs::proxy_get_children(map<string, cliproxy*>& c)
{
    c["connect_viewer"]    = new memfunc_proxy<svs>(this, &svs::cli_connect_viewer);
    c["connect_viewer"]->set_help("Connect to a running viewer.")
    .add_arg("PORT", "TCP port (or file socket path in Linux) to connect to.")
    ;
    
    c["disconnect_viewer"] = new memfunc_proxy<svs>(this, &svs::cli_disconnect_viewer);
    c["disconnect_viewer"]->set_help("Disconnect from viewer.");
    
    c["filters"]           = &get_filter_table();
    c["commands"]          = &get_command_table();

#ifdef ENABLE_ROS
    c["objectives"] = &get_objective_table();
    c["ros"] = ri;
#endif

    for (size_t j = 0, jend = state_stack.size(); j < jend; ++j)
    {
        c[state_stack[j]->get_name()] = state_stack[j];
    }
}

bool svs::do_cli_command(const vector<string>& args, string& output)
{
    stringstream ss;
    vector<string> rest;
    
    if (args.size() < 2)
    {
        output = "specify path\n";
        return false;
    }
    
    for (size_t i = 2, iend = args.size(); i < iend; ++i)
    {
        rest.push_back(args[i]);
    }
    
    proxy_use(args[1], rest, ss);
    output = ss.str();
    return true;
}

void svs::cli_connect_viewer(const vector<string>& args, ostream& os)
{
    if (args.empty())
    {
        os << "specify socket path" << endl;
        return;
    }
    if (draw->connect(args[0]))
    {
        os << "connection successful" << endl;
        for (size_t i = 0, iend = state_stack.size(); i < iend; ++i)
        {
            state_stack[i]->get_scene()->refresh_draw();
        }
    }
    else
    {
        os << "connection failed" << endl;
    }
}

void svs::cli_disconnect_viewer(const vector<string>& args, ostream& os)
{
    draw->disconnect();
}
