#include <handle_gui.h>

// C++ standard headers
#include <exception>
#include <string>

// ROS headers
#include <ros/ros.h>

// PAL includes
#include <pal_web_msgs/WebGoTo.h>

// =====================================

handle_gui::handle_gui(ros::NodeHandle    *nodehandle):  nh_(*nodehandle)
{
  // Initialize joint state variables.
  handle_gui::init();
}

// ###################################
void handle_gui::cook_web_msg(const std::string    &web_name,
                              const uint8_t        &web_type)
{
  // Cooking voice cmd.
  web_msg_.type  = web_type; // https://github.com/pal-robotics/pal_msgs/blob/indigo-devel/pal_web_msgs/msg/WebGoTo.msg
  web_msg_.value = web_name;
}

void handle_gui::init()
{
  std::cout << "Initializing handle_gui ..." << std::endl;

  // Initialize user_input subscriber.
  web_go_to_pub_ = nh_.advertise<pal_web_msgs::WebGoTo>("/web/go_to",10);

  std::cout << "handle_gui initialized!" << std::endl;
}


bool handle_gui::set_web_go_to(const std::string    &web_name,
                               const uint8_t        &web_type)
{
  handle_gui::cook_web_msg(web_name, web_type);

  // Send goal to the client.
  ROS_INFO_STREAM("Robot: Publishing in -> " << web_name);
  web_go_to_pub_.publish(web_msg_);

  return true;
}
