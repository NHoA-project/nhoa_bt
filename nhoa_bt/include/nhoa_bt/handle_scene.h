#ifndef __handle_scene_H_INCLUDED__
#define __handle_scene_H_INCLUDED__

// C++ standard headers
#include <exception>
#include <string>

// ROS headers
#include <ros/ros.h>

// UPO includes
#include <upo_sgg_msgs/SceneDigest.h>

/* This class encapsulates the PAL ARI's GUI functionalities. */

class handle_scene
{
private:
    //vars
    // ROS stuff
    ros::NodeHandle     nh_;

    // Scene Digest stuff.
    ros::Subscriber             scene_digest_sub_;

    // =======
    //functions
    // Initializing.
    void init();

    // Callback function.
    void scene_digest_callback(const upo_sgg_msgs::SceneDigest&  scene_digest);

public:
    //vars
    upo_sgg_msgs::SceneDigest   scene_digest_msg_;

    // =======
    //functions
    handle_scene(ros::NodeHandle    *nodehandle);
};
#endif // __handle_scene_H_INCLUDED__