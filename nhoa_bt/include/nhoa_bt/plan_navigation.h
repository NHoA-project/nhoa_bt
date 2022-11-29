#ifndef __PLANNING_NAVIGATION_H_INCLUDED__
#define __PLANNING_NAVIGATION_H_INCLUDED__

// ROS headers
#include <ros/ros.h>
#include "tf/transform_listener.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "nav_msgs/Odometry.h"

// STD includes.
#include <mutex>

// =====

class plan_navigation
{
private:
    //vars
    // ROS stuff
    ros::NodeHandle     nh_;
    ros::Subscriber     odom_sub_;

    // Actionlib stuff.
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client_;

    // Move base stuff.
    move_base_msgs::MoveBaseGoal goal_;
    bool                         action_status_;

    // Odometry callback stuff.
    nav_msgs::Odometry  odom_;
    std::mutex          mutex_;

    // =======
    //functions
    // Initializing.
    void init();

    // Cook navigation.
    void cook_navigation(const std::vector<double>    &navigation_goal);

    // Odom callback.
    void odom_callback(const nav_msgs::Odometry &odom_msg);

public:
    //vars

    // =======
    //functions
    plan_navigation(ros::NodeHandle    *nodehandle);

    // Set 2D navigation goal (position + rotation).
    bool set_navigation_goal(const std::vector<double>    &navigation_goal);

    // Set 2D rotation.
    bool set_rotation(const double &rotation);
};
#endif // __PLANNING_NAVIGATION_H_INCLUDED__