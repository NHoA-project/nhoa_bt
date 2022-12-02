#ifndef __handle_gui_H_INCLUDED__
#define __handle_gui_H_INCLUDED__

// C++ standard headers
#include <exception>
#include <string>

// ROS headers
#include <ros/ros.h>

// PAL includes
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
    std::vector <std::string>   web_list_ = {"web1",
                                             "web2"};

    // =======
    //functions
    // Initializing.
    void init();

    // Cook voice.
    void cook_web_msg(const std::string    &web_name,
                      const uint8_t        &web_type);

public:
    //vars

    // =======
    //functions
    handle_gui(ros::NodeHandle    *nodehandle);

    // Set predefined motion.
    bool set_web_go_to(const std::string    &web_name,
                       const uint8_t        &web_type);
};
#endif // __handle_gui_H_INCLUDED__