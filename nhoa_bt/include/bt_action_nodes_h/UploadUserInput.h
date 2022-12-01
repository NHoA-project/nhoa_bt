#ifndef UploadUserInput_H_INCLUDED_
#define UploadUserInput_H_INCLUDED_

// GENERAL INCLUDES
#include <iostream>
#include <math.h>

// ROS INCLUDES
#include <ros/ros.h>

// BT includes.
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// NHOA_BT INCLUDES
#include <handle_voice.h>


/* TODO: This BT Action node encapsulates all the functionalities
         to handle the user conversation answers.*/

class UploadUserInput : public BT::CoroActionNode
{
  public:                                                                                                     

    // Shared program resources.
    handle_voice* voice_;

    // Bool flag.
    bool success_ = false;

    // User input stuff.
    std::string user_input;
    bool waiting_user = true;

    // ==============

    UploadUserInput(const std::string& name, const BT::NodeConfiguration& config) : 
    BT::CoroActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        // This action has a single input port called "message"
        // Any port must have a name. The type is optional.
        return { {BT::InputPort<std::string>("_conversation_mode")},
                 {BT::InputPort<std::size_t>("_iteration")}};
    }

    void init(handle_voice*  input_voice)
    {
      std::cout << "### Initializing UploadUserInput! ###" << std::endl;

      voice_   = input_voice;
    }

    // You must override the virtual function tick()
    BT::NodeStatus      tick() override;
    void                cleanup(bool halted);
    void                halt() override;

    // =================================================== 
    // Additional functionalities.

    // Inputs the Questionnaire answers.
    bool getQuestionnaireInput(const std::size_t  &iteration);

    // TODO: Sent Answers to the database.
};
#endif // UploadUserInput_H_INCLUDED_