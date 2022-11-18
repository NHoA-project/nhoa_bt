#ifndef DrawUserAttention_H_INCLUDED_
#define DrawUserAttention_H_INCLUDED_

// GENERAL INCLUDES
#include <iostream>
#include <math.h>

// ROS INCLUDES
#include <ros/ros.h>

// BT includes.
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

/* TODO: This BT Action node outputs the "motion_name" and 
         "voice command" to grap the user attention.*/


class DrawUserAttention : public BT::CoroActionNode
{
  public:   

    // TODO: Predefined messages.
    std::vector<std::string> motion_names = {"nod",
                                             "look around",
                                             "show left",
                                             "show right",
                                             "bow"};
    std::vector<std::string> voice_cmds = {"Hi. Good morning!",
                                           "Hello! Can you hear me?",
                                           "Hey, I am here! Close to the kitchen.",
                                           "Oh, wow. I would really love a chat."};                                         

    // Bool flag.
    bool success_ = false;


    // ==============

    DrawUserAttention(const std::string& name, const BT::NodeConfiguration& config) : 
    BT::CoroActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        // This action has a single input port called "message"
        // Any port must have a name. The type is optional.
        return { {BT::InputPort<std::size_t>("_iteration")}, 
                 {BT::OutputPort<std::size_t>("iteration_")},
                 {BT::OutputPort<std::string>("motion_name_")},
                 {BT::OutputPort<std::string>("voice_cmd_")} };
    }

    void init()
    {
      std::cout << "### Initializing DrawUserAttention! ###" << std::endl;
    }

    // You must override the virtual function tick()
    BT::NodeStatus      tick() override;
    void                cleanup(bool halted);
    void                halt() override;

    // =================================================== 
    // Additional functionalities.

    // Outputs the "motion_name" and "voice command" to grap the user attention.
    bool drawUserAttention(const std::size_t  &iteration);
};
#endif // DrawUserAttention_H_INCLUDED_