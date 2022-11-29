#ifndef __plan_voice_cmd_H_INCLUDED__
#define __plan_voice_cmd_H_INCLUDED__

// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// PAL includes
#include <pal_interaction_msgs/TtsAction.h>

// C++ standard headers
#include <cstdlib>

/* This class encapsulates the voice engine functionalities. */

class plan_voice_cmd
{
private:
    //vars
    // ROS stuff
    ros::NodeHandle     nh_;

    // Actionlib stuff.
    actionlib::SimpleActionClient<pal_interaction_msgs::TtsAction>           client_;
    
    // Play motion stuff.
    bool                                 action_status_;
    pal_interaction_msgs::TtsGoal        goal_;

    // =======
    //functions
    // Initializing.
    void init();

    // Cook voice.
    void cook_voice_cmd(const std::string                 &voice_cmd);

public:
    //vars

    // =======
    //functions
    plan_voice_cmd(ros::NodeHandle    *nodehandle);

    // Set predefined motion.
    bool set_voice_cmd(const std::string    &voice_cmd);
};
#endif // __plan_voice_cmd_H_INCLUDED__