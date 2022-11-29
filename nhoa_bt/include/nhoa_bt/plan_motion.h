#ifndef __PLAN_MOTION_H_INCLUDED__
#define __PLAN_MOTION_H_INCLUDED__

// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// PAL includes
#include <play_motion_msgs/PlayMotionAction.h>

// C++ standard headers
#include <cstdlib>

/* This class encapsulates the planning of the Uper Body predefined motions. */

class plan_motion
{
private:
    //vars
    // ROS stuff
    ros::NodeHandle     nh_;

    // Actionlib stuff.
    actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction>           client_;
    
    // Play motion stuff.
    bool                                    action_status_;
    play_motion_msgs::PlayMotionGoal        goal_;

    // =======
    //functions
    // Initializing.
    void init();

    // Cook play motion.
    void cook_goal(const std::string                 &motion_name);

public:
    //vars

    // =======
    //functions
    plan_motion(ros::NodeHandle    *nodehandle);

    // Set predefined motion.
    bool set_motion(const std::string    &motion_name);
};
#endif // __PLAN_MOTION_H_INCLUDED__