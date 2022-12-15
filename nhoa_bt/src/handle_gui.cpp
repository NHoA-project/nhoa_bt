#include <handle_gui.h>

// C++ standard headers
#include <exception>
#include <string>

// ROS headers
#include <ros/ros.h>

// PAL includes
#include <pal_interaction_msgs/Input.h>
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

  // Set maximum iteration.
  max_iter_ = nh_.param("questionnaire_max_iterations", 5);

  // Initialize web to go publisher.
  web_go_to_pub_ = nh_.advertise<pal_web_msgs::WebGoTo>("/web/go_to",10);

  // Initialize user_input subscriber.
  user_input_sub_ = nh_.subscribe("/user_input", 1, &handle_gui::user_input_callback, this);

  std::cout << "handle_gui initialized!" << std::endl;
}


bool handle_gui::set_web_go_to(const std::string    &gui_type,
                               const std::size_t    &iteration,
                               const uint8_t        &web_type)
{
  if(gui_type.compare("questionnaire") == 0 )
  {
    if(iteration < max_iter_)
    {
      handle_gui::cook_web_msg(web_list_[iteration], web_type);

      // Send goal to the client.
      ROS_INFO_STREAM("Robot: Publishing in -> " << web_list_[iteration]);
      web_go_to_pub_.publish(web_msg_);
      success_ = true;
    }
    else
    {
      success_ =  false;
    }
  }
  else if( gui_type.compare("logo") == 0 )
  {
    handle_gui::cook_web_msg(web_logo_, web_type);

      // Send goal to the client.
      ROS_INFO_STREAM("Robot: Publishing in -> " << web_logo_);
      web_go_to_pub_.publish(web_msg_);
      success_ = true;
  }
  return success_;
}

void handle_gui::user_input_callback(const pal_interaction_msgs::Input&  user_input)
{
  // Debug.
  // std::cout << "User input -> " << user_input.action << std::endl; 
  user_input_flag_  = true;
  user_input_msg_   = user_input.action; // TODO: Modify the message type.

}
