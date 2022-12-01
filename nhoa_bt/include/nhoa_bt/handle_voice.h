#ifndef __handle_voice_H_INCLUDED__
#define __handle_voice_H_INCLUDED__

// C++ standard headers
#include <exception>
#include <string>
#include "std_msgs/String.h"

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// PAL includes
#include <pal_interaction_msgs/TtsAction.h>
#include <pal_interaction_msgs/Input.h>

// C++ standard headers
#include <cstdlib>
#include <mutex>

/* This class encapsulates the voice engine functionalities. */

class handle_voice
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

    // User input stuff.
    ros::Subscriber     user_input_sub_;
    std::mutex          mutex_;

    // =======
    //functions
    // Initializing.
    void init();

    // Cook voice.
    void cook_voice_cmd(const std::string   &voice_cmd);

    // User input callback.
    void user_input_callback(const pal_interaction_msgs::Input&  user_input);

public:
    //vars
    std::string         user_input_msg_;

    // =======
    //functions
    handle_voice(ros::NodeHandle    *nodehandle);

    // Set predefined motion.
    bool set_voice_cmd(const std::string    &voice_cmd);
};
#endif // __handle_voice_H_INCLUDED__