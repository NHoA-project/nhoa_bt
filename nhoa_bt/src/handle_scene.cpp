#include <handle_scene.h>

// C++ standard headers
#include <exception>
#include <string>

// ROS headers
#include <ros/ros.h>

// UPO includes
#include <upo_sgg_msgs/SceneDigest.h>

// =====================================

handle_scene::handle_scene(ros::NodeHandle    *nodehandle):  nh_(*nodehandle)
{
  // Initialize joint state variables.
  handle_scene::init();
}

// ###################################

void handle_scene::init()
{
  std::cout << "Initializing handle_scene ..." << std::endl;

  // Initialize user_input subscriber.
  scene_digest_sub_ = nh_.subscribe("/scene_digest", 1, &handle_scene::scene_digest_callback, this);

  std::cout << "handle_scene initialized!" << std::endl;
}


void handle_scene::scene_digest_callback(const upo_sgg_msgs::SceneDigest&  scene_digest)
{
  scene_digest_msg_ = scene_digest;
}
