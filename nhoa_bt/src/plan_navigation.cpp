#include <plan_navigation.h>

// ROS headers
#include <ros/ros.h>

// =====================================

plan_navigation::plan_navigation(ros::NodeHandle    *nodehandle):  nh_(*nodehandle),
                                                                   client_("/move_base", true)
{
  // Initialize joint state variables.
  plan_navigation::init();
}

// ###################################

void plan_navigation::cook_navigation(const std::vector<double>    &navigation_goal)
{
  // Debug.
  std::cout << "Navigation goal (x, y, theta) -> [" << navigation_goal[0] << ", " << navigation_goal[1] << ", " << navigation_goal[2] << "]" << std::endl;

  // Cooking navigation.
    goal_.target_pose.header.frame_id     = "map";                          // Gazebo origin.
    goal_.target_pose.header.stamp        = ros::Time::now();
    goal_.target_pose.pose.position.x     = navigation_goal[0];
    goal_.target_pose.pose.position.y     = navigation_goal[1];
    goal_.target_pose.pose.orientation.z  = sin(navigation_goal[2] / 2.0);  // rad
    goal_.target_pose.pose.orientation.w  = cos(navigation_goal[2] / 2.0);  // rad

}

void plan_navigation::init()
{
  std::cout << "Initializing plan_navigation ..." << std::endl;

  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    // return EXIT_FAILURE;
  }
  
  // ROS_INFO("Waiting for Action Server ...");
  std::cout << "I'm here ... " << std::endl;
  // client_.waitForServer(); # Should be implemented w/ Real Ari.

  // Initialize subscriber.
  odom_sub_ = nh_.subscribe("/mobile_base_controller/odom", 1, &plan_navigation::odom_callback, this);

  std::cout << "plan_navigation initialized!" << std::endl;
}

void plan_navigation::odom_callback(const nav_msgs::Odometry &odom_msg)
{
  mutex_.lock();
  odom_ = odom_msg;
  mutex_.unlock();
}

bool plan_navigation::set_navigation_goal(const std::vector<double>    &navigation_goal)
{

  // plan_navigation::cook_navigation(navigation_goal);

  // Send goal to "navigation client".
  ROS_INFO_STREAM("Sending navigation goal!");  
  // client_.sendGoal(goal_);

  // Waiting for action status.
  ROS_INFO("Waiting for result ...");
  // action_status_ = client_.waitForResult(ros::Duration(30.0));

  // Get action status.
  // actionlib::SimpleClientGoalState state_ = client_.getState();

  // if ( action_status_ )
  // {
  //     ROS_INFO_STREAM("Action finished successfully with state: " << state_.toString());
  // }
  // else
  // {
  //     ROS_ERROR_STREAM("Action failed with state: " << state_.toString());
  // }
  // return action_status_;

  return true;
}

bool plan_navigation::set_rotation(const double &rotation)
{
  // plan_navigation::cook_navigation({odom_.pose.pose.position.x, odom_.pose.pose.position.y, 2*asin(odom_.pose.pose.position.z) + rotation});

  // Send goal to "navigation client".
  ROS_INFO_STREAM("Sending rotation goal:" << rotation);  
  // client_.sendGoal(goal_);

  // Waiting for action status.
  ROS_INFO("Waiting for result ...");
  // action_status_ = client_.waitForResult(ros::Duration(30.0));

  // // Get action status.
  // actionlib::SimpleClientGoalState state_ = client_.getState();

  // if ( action_status_ )
  // {
  //     ROS_INFO_STREAM("Action finished successfully with state: " << state_.toString());
  // }
  // else
  // {
  //     ROS_ERROR_STREAM("Action failed with state: " << state_.toString());
  // }
  // return action_status_;

  return true;
}