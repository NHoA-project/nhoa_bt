#ifndef __PLAN_HEAD_MOTION_H_INCLUDED__
#define __PLAN_HEAD_MOTION_H_INCLUDED__

// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/PointHeadAction.h>

// C++ standard headers
#include <cstdlib>

// UPO includes.
#include "nhoa_head_following_action/HeadFollowingAction.h"

/* This class encapsulates the ARI's head planning. */

class plan_head_motion
{
private:
    //vars
    // ROS stuff
    ros::NodeHandle     nh_;
    ros::Subscriber     head_following_sub_;

    // Actionlib stuff.
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>    follow_joint_traj_client_;
    actionlib::SimpleActionClient<control_msgs::PointHeadAction>                point_head_client_;

    bool                                                                        action_status_;
    
    // Follow Joint Trajectory stuff.
    control_msgs::FollowJointTrajectoryGoal follow_jont_traj_goal_;
    std::vector <std::string>               head_joint_names_ = {"head_1_joint", "head_2_joint"}; 

    // Point Head stuff.
    control_msgs::PointHeadGoal             point_head_goal_;

    // Head Following stuff.
    actionlib::SimpleActionClient<nhoa_head_following_action::HeadFollowingAction>      head_following_client_;
    nhoa_head_following_action::HeadFollowingGoal                                       head_following_goal_;

    // =======
    //functions
    // Cook head controller Follow Joint Trajectory goal.
    void cook_follow_joint_traj(const std::vector<double>            &joint);

    // Cook head controller Point Head Action goal.
    void cook_point_head_goal(const geometry_msgs::PointStamped      &point);

    // Cook head controller Head Following Action goal.
    void cook_head_following_goal();


    // Head following feedback callback.
    void head_following_feedback_callback(const nhoa_head_following_action::HeadFollowingActionFeedback &msg);

    // Initializing.
    void init();

    // Initializing Follow Joint Trajectory goal.
    void init_fjt_goal();

public:
    //vars
    nhoa_head_following_action::HeadFollowingActionFeedback         head_following_feedback_;

    // =======
    //functions
    plan_head_motion(ros::NodeHandle    *nodehandle);

    // Set head joint motion.
    bool set_joint_trajectory_goal(const std::vector<double>    &joint); // joint [pan, tilt].

    // Set head folowing motion.
    bool set_head_following();

    // Set head point motion.
    bool set_point_head_goal(const geometry_msgs::PointStamped       &point);

};
#endif // __PLAN_HEAD_MOTION_H_INCLUDED__