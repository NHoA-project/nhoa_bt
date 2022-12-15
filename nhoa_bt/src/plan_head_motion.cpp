#include <plan_head_motion.h>

// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// C++ standard headers
#include <cstdlib>

// =====================================

plan_head_motion::plan_head_motion(ros::NodeHandle    *nodehandle):  nh_(*nodehandle),
                                                                     follow_joint_traj_client_("head_controller/follow_joint_trajectory", true),
                                                                     point_head_client_("head_controller/point_head_action", true),
                                                                     head_following_client_("HeadFollowing", true)
{
  // Initialize joint state variables.
  plan_head_motion::init();
}

// ###################################


void plan_head_motion::cook_head_following_goal()
{
  // Cooking navigation.
  head_following_goal_.target_id = "-1";
}

void plan_head_motion::cook_follow_joint_traj(const std::vector<double>    &joint)
{

  for (size_t i=0; i<2; i++)
  {
    follow_jont_traj_goal_.trajectory.points[0].positions[i]   = joint[i];
    follow_jont_traj_goal_.trajectory.points[0].velocities[i]  = 0.0;
  }

  follow_jont_traj_goal_.trajectory.points[0].time_from_start  = ros::Duration(10.0);

}

void plan_head_motion::cook_point_head_goal(const geometry_msgs::PointStamped &point)
{
  // The pointHeadGoal consists in making the Z axis of the cameraFrame to point towards the pointStamped
  point_head_goal_.pointing_frame = "/head_front_camera_color_optical_frame"; // TODO: Modify that frame.
  point_head_goal_.pointing_axis.x = 0.0;
  point_head_goal_.pointing_axis.y = 0.0;
  point_head_goal_.pointing_axis.z = 1.0;
  point_head_goal_.min_duration = ros::Duration(0.1);
  point_head_goal_.max_velocity = 0.5;
  point_head_goal_.target = point;
}

void plan_head_motion::head_following_feedback_callback(const nhoa_head_following_action::HeadFollowingActionFeedback &msg)
{
  head_following_feedback_ = msg;
  if(head_following_feedback_.feedback.found)
    std::cout << " Person detected -> " << "TRUE" << std::endl;
  else
    std::cout << " Person detected -> " << "FALSE" << std::endl;

}


void plan_head_motion::init()
{
  std::cout << "Initializing plan_head_motion ..." << std::endl;

  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    // return EXIT_FAILURE;
  }
  
  // ROS_INFO("Waiting for Action Server ...");
  follow_joint_traj_client_.waitForServer();
  point_head_client_.waitForServer();
  // head_following_client_.waitForServer();

  // Init Follow Joint Trajectory goal.
  plan_head_motion::init_fjt_goal();

  // Init Head Following subscriber.
  head_following_sub_    = nh_.subscribe("/HeadFollowing/feedback", 1, &plan_head_motion::head_following_feedback_callback, this);

  std::cout << "plan_head_motion initialized!" << std::endl;
}

void plan_head_motion::init_fjt_goal()
{
  follow_jont_traj_goal_.trajectory.points.resize(1);
  follow_jont_traj_goal_.trajectory.points[0].positions.resize(2);
  follow_jont_traj_goal_.trajectory.points[0].velocities.resize(2);
  follow_jont_traj_goal_.trajectory.joint_names.resize(2);
  follow_jont_traj_goal_.trajectory.joint_names[0] = head_joint_names_[0];
  follow_jont_traj_goal_.trajectory.joint_names[1] = head_joint_names_[1];
}

bool plan_head_motion::set_joint_trajectory_goal(const std::vector<double>    &joint)
{

  plan_head_motion::cook_follow_joint_traj(joint);

  auto goal_state = follow_joint_traj_client_.sendGoalAndWait(follow_jont_traj_goal_,ros::Duration(20.0)); //send goal and wait until complete or timout exceeded
  if (goal_state.state_==goal_state.SUCCEEDED)
  {
    return true;
  }
  else
  {
      ROS_ERROR("The head action has not succeeded");
      std::cout << "The goal state is: " << goal_state.toString() <<std::endl;
      return false;
  }
}

bool plan_head_motion::set_point_head_goal(const geometry_msgs::PointStamped       &point)
{

  plan_head_motion::cook_point_head_goal(point);

  auto goal_state = point_head_client_.sendGoalAndWait(point_head_goal_,ros::Duration(10.0)); //send goal and wait until complete or timout exceeded
  if (goal_state.state_==goal_state.SUCCEEDED)
  {
    return true;
  }
  else
  {
      ROS_ERROR("The head action has not succeeded");
      std::cout << "The goal state is: " << goal_state.toString() <<std::endl;
      return false;
  }
}

bool plan_head_motion::set_head_following()
{

  plan_head_motion::cook_head_following_goal();

  // Send goal to "navigation client".
  ROS_INFO_STREAM("Sending head following goal!");  
  head_following_client_.sendGoal(head_following_goal_);

  return true;
}