#include <handle_voice.h>

// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// PAL includes
#include <pal_interaction_msgs/Input.h>

// C++ standard headers
#include <cstdlib>

// =====================================

handle_voice::handle_voice(ros::NodeHandle    *nodehandle):  nh_(*nodehandle),
                                                             client_("tts", true)
{
  // Initialize joint state variables.
  handle_voice::init();
}

// ###################################
void handle_voice::cook_voice_cmd(const std::string    &voice_cmd)
{
  // Cooking voice cmd.
  goal_.rawtext.text    = voice_cmd;
  goal_.rawtext.lang_id = "es_ES";
}

void handle_voice::init()
{
  std::cout << "Initializing handle_voice ..." << std::endl;

  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    // return EXIT_FAILURE;
  }
  
  // ROS_INFO("Waiting for Action Server ...");
  client_.waitForServer();

  std::cout << "handle_voice initialized!" << std::endl;
}


bool handle_voice::set_voice_cmd(const std::string    &voice_cmd)
{
  handle_voice::cook_voice_cmd(voice_cmd);

  // Send goal to the client.
  ROS_INFO_STREAM("Sending robot's voice cmd -> " << voice_cmd);  
  client_.sendGoal(goal_);

  // Waiting for action status.
  ROS_INFO("Waiting for result ...");
  action_status_ = client_.waitForResult(ros::Duration(30.0));

  // Get action status.
  actionlib::SimpleClientGoalState state_ = client_.getState();

  if ( action_status_ )
  {
      ROS_INFO_STREAM("Action finished successfully with state: " << state_.toString());
  }
  else
  {
      ROS_ERROR_STREAM("Action failed with state: " << state_.toString());
  }
  return action_status_;
  // return true;
}
