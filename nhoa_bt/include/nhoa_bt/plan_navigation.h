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

// UPO includes.
#include "nhoa_approach_action/ApproachAction.h"

// =====

class plan_navigation
{
private:
    //vars
    // ROS stuff
    ros::NodeHandle     nh_;
    ros::Subscriber     odom_sub_;
    ros::Subscriber     approach_feedback_sub_;
    ros::Subscriber     approach_result_sub_;

    // Actionlib stuff.
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>       client_;
    actionlib::SimpleActionClient<nhoa_approach_action::ApproachAction> approach_client_;

    // Move base stuff.
    move_base_msgs::MoveBaseGoal goal_;
    bool                         action_status_;

    // Approach navigation stuff.
    double                                        approach_distance_;
    double                                        approach_distance_threshold_;
    nhoa_approach_action::ApproachGoal            approach_goal_;
    nhoa_approach_action::ApproachActionFeedback  approach_feedback_;
    nhoa_approach_action::ApproachActionResult     approach_result_;
    bool                                          is_approach_reached_ = false;


    // Odometry callback stuff.
    nav_msgs::Odometry  odom_;
    std::mutex          mutex_;

    // =======
    //functions

    // Approach feedback callback.
    void approach_feedback_callback(const nhoa_approach_action::ApproachActionFeedback &msg);

    // Approach result callback.
    void approach_result_callback(const nhoa_approach_action::ApproachActionResult &msg);

    // Initializing.
    void init();

    // Cook approach navigation.
    void cook_approach_navigation();

    // Cook navigation.
    void cook_navigation(const std::vector<double>    &navigation_goal);

    // Odom callback.
    void odom_callback(const nav_msgs::Odometry &odom_msg);

public:
    //vars

    // =======
    //functions
    plan_navigation(ros::NodeHandle    *nodehandle);

    // Cancel action.
    bool cancel_goal();

    // Check approach distance.
    bool check_approach_distance();

    // Set approach navigation.
    bool set_approach_navigation();

    // Set 2D navigation goal (position + rotation).
    bool set_navigation_goal(const std::vector<double>    &navigation_goal);

    // Set 2D rotation.
    bool set_rotation(const double &rotation);
};
#endif // __PLANNING_NAVIGATION_H_INCLUDED__