#ifndef __PLANNING_MOTION_H_INCLUDED__
#define __PLANNING_MOTION_H_INCLUDED__

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

class plan_motion
{
private:
    //vars
    // ROS stuff
    ros::NodeHandle     nh_;

    // Actionlib stuff.
    actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction>   client_;

    // Play motion stuff.
    play_motion_msgs::PlayMotionGoal goal_;
    bool                             action_status_;

    // =======
    //functions
    // Initializing.
    void init();

    // Cook motion.
    void cook_motion(const std::string    &motion_name);

public:
    //vars

    // =======
    //functions
    plan_motion(ros::NodeHandle    *nodehandle);

    bool set_motion(const std::string    &motion_name);
};
#endif // __PLANNING_MOTION_H_INCLUDED__