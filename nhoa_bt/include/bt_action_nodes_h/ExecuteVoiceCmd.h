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

// RECYCLA INCLUDES
#include <plan_motion.h>

/* This BT action node encapsulates the PAL ROBOTICS ARI "tts" 
   speech functionality.*/

class ExecuteVoiceCmd : public BT::CoroActionNode
{
  public:  

    // Bool flag.
    bool success_ = false;

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
                //  {BT::OutputPort<std::string>("move_group_")}};
    }

    void init()
    {
      std::cout << "### Initializing ExecuteVoiceCmd! ###" << std::endl;
    }

    // You must override the virtual function tick()
    BT::NodeStatus      tick() override;
    void                cleanup(bool halted);
    void                halt() override;

    // =================================================== 

    // Execute Cartersian Path by specifying a list of waypoints to the EE
    // to go through.
    bool executeVoiceCmd(const std::string  &motion_name);
};
#endif // ExecuteVoiceCmd_H_INCLUDED_