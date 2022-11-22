#include <plan_motion.h>

// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <play_motion_msgs/PlayMotionAction.h>

// C++ standard headers
#include <cstdlib>

// =====================================

plan_motion::plan_motion(ros::NodeHandle    *nodehandle):  nh_(*nodehandle),
                                                           client_("/play_motion", true)
{
  // Initialize joint state variables.
  plan_motion::init();
}

// ###################################

void plan_motion::cook_motion(const std::string    &motion_name)
{
  // Cooking motion.
  goal_.motion_name = motion_name;
  goal_.skip_planning = false;
  goal_.priority = 0;
}

void plan_motion::init()
{
  std::cout << "Initializing plan_motion ..." << std::endl;

  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    // return EXIT_FAILURE;
  }
  
  // ROS_INFO("Waiting for Action Server ...");
  client_.waitForServer();
}

bool plan_motion::set_motion(const std::string    &motion_name)
{
  plan_motion::cook_motion(motion_name);

  // Send goal to the client.
  ROS_INFO_STREAM("Sending goal with motion: " << motion_name);  
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
}