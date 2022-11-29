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

/* This class encapsulates the ARI's head planning. */

class plan_head_motion
{
private:
    //vars
    // ROS stuff
    ros::NodeHandle     nh_;

    // Actionlib stuff.
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>    follow_joint_traj_client_;
    actionlib::SimpleActionClient<control_msgs::PointHeadAction>                point_head_client_;
    bool                                                                        action_status_;
    
    // Follow Joint Trajectory stuff.
    control_msgs::FollowJointTrajectoryGoal follow_jont_traj_goal_;
    std::vector <std::string>               head_joint_names_ = {"head_1_joint", "head_2_joint"}; 

    // Point Head stuff.
    control_msgs::PointHeadGoal             point_head_goal_;

    // =======
    //functions
    // Cook head controller Follow Joint Trajectory goal.
    void cook_follow_joint_traj(const std::vector<double>            &joint);

    // Cook head controller Point Head Action goal.
    void cook_point_head_goal(const geometry_msgs::PointStamped      &point);

    // Initializing.
    void init();

    // Initializing Follow Joint Trajectory goal.
    void init_fjt_goal();

public:
    //vars

    // =======
    //functions
    plan_head_motion(ros::NodeHandle    *nodehandle);

    // Set head joint motion.
    bool set_joint_trajectory_goal(const std::vector<double>    &joint); // joint [pan, tilt].

    // Set head point motion.
    bool set_point_head_goal(const geometry_msgs::PointStamped       &point);
};
#endif // __PLAN_HEAD_MOTION_H_INCLUDED__