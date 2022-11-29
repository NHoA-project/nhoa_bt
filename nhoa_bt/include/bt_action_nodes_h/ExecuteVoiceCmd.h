#ifndef ExecuteVoiceCmd_H_INCLUDED_
#define ExecuteVoiceCmd_H_INCLUDED_

// GENERAL INCLUDES
#include <iostream>
#include <math.h>

// ROS INCLUDES
#include <ros/ros.h>

// BT includes.
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// NHOA_BT INCLUDES
#include <plan_voice_cmd.h>

/* This BT action node encapsulates the PAL ROBOTICS ARI "tts" 
   speech functionality.*/

class ExecuteVoiceCmd : public BT::CoroActionNode
{
  public:  

    // Shared program resources.
    plan_voice_cmd* voice_;

    // ROS stuff.
    ros::NodeHandle             nh_;    
        
    // MoveIt! stuff.
    bool success_                 = false;

    // =================================================== 

    ExecuteVoiceCmd(const std::string& name, const BT::NodeConfiguration& config) : 
    BT::CoroActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        // This action has a single input port called "message"
        // Any port must have a name. The type is optional.
        return { BT::InputPort<std::string>("_voice_cmd") };
    }

    void init(ros::NodeHandle*  input_nh,
              plan_voice_cmd*   input_voice_cmd)
    {
      std::cout << "### Initializing ExecuteVoiceCmd! ###" << std::endl;

      nh_       = *input_nh;
      voice_    = input_voice_cmd;
    }

    // You must override the virtual function tick()
    BT::NodeStatus      tick() override;
    void                cleanup(bool halted);
    void                halt() override;

    // =================================================== 

    // Execute Cartersian Path by specifying a list of waypoints to the EE
    // to go through.
    bool executeVoiceCmd(const std::string  &voice_cmd);
};
#endif // ExecuteVoiceCmd_H_INCLUDED_