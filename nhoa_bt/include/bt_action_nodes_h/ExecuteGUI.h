#ifndef ExecuteGUI_H_INCLUDED_
#define ExecuteGUI_H_INCLUDED_

// GENERAL INCLUDES
#include <iostream>

// ROS INCLUDES
#include <ros/ros.h>

// BT includes.
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// NHOA_BT INCLUDES
#include <handle_gui.h>

/* This BT action node encapsulates the PAL ROBOTICS ARI GUI 
   functionalities.*/

class ExecuteGUI : public BT::CoroActionNode
{
  public:  

    // Shared program resources.
    handle_gui* gui_;

    // ROS stuff.
    ros::NodeHandle             nh_;    
        
    // MoveIt! stuff.
    bool success_                 = false;

    // =================================================== 

    ExecuteGUI(const std::string& name, const BT::NodeConfiguration& config) : 
    BT::CoroActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        // This action has a single input port called "message"
        // Any port must have a name. The type is optional.
        return { {BT::InputPort<std::size_t>("_iteration")},
                 {BT::InputPort<std::string>("_gui_type")},
                 {BT::InputPort<int>("_web_type")} };
    }

    void init(ros::NodeHandle*  input_nh,
              handle_gui*       input_gui)
    {
      std::cout << "### Initializing ExecuteGUI! ###" << std::endl;

      nh_       = *input_nh;
      gui_      = input_gui;
    }

    // You must override the virtual function tick()
    BT::NodeStatus      tick() override;
    void                cleanup(bool halted);
    void                halt() override;

    // =================================================== 

    // Sent
    bool executeGUI(const std::string  &gui_type,
                    const std::size_t  &iteration,
                    const int          &web_type);
};
#endif // ExecuteGUI_H_INCLUDED_