#ifndef __handle_gui_H_INCLUDED__
#define __handle_gui_H_INCLUDED__

// C++ standard headers
#include <exception>
#include <string>

// ROS headers
#include <ros/ros.h>

// PAL includes
#include <pal_interaction_msgs/Input.h>
#include <pal_web_msgs/WebGoTo.h>

/* This class encapsulates the PAL ARI's GUI functionalities. */

class handle_gui
{
private:
    //vars
    // ROS stuff
    ros::NodeHandle     nh_;

    // Web go to stuff.
    ros::Publisher              web_go_to_pub_;
    pal_web_msgs::WebGoTo       web_msg_;
    std::vector <std::string>   web_list_ = {"nhoa_Q1",
                                             "nhoa_Q2",
                                             "nhoa_Q3"};
    std::string                 web_logo_ = "nhoa_image";
    
    // User input stuff.
    ros::Subscriber     user_input_sub_;

    // Boolean stuff.
    bool    success_ = false;

    // =======
    //functions
    // Initializing.
    void init();

    // Cook voice.
    void cook_web_msg(const std::string    &web_name,
                      const uint8_t        &web_type);

    // User input callback.
    void user_input_callback(const pal_interaction_msgs::Input&  user_input);

public:
    //vars
    std::string         user_input_msg_;
    bool                user_input_flag_        = false;
    size_t              max_iter_;

    // =======
    //functions
    handle_gui(ros::NodeHandle    *nodehandle);

    // Set predefined motion.
    bool set_web_go_to(const std::string    &gui_type,
                       const std::size_t    &iteration,
                       const uint8_t        &web_type);
};
#endif // __handle_gui_H_INCLUDED__