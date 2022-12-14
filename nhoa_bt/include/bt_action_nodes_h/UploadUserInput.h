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
#include <handle_gui.h>
#include <handle_hri.h>


/* TODO: This BT Action node encapsulates all the functionalities
         to handle the user conversation answers.*/

class UploadUserInput : public BT::CoroActionNode
{
  public:                                                                                                     

    // Shared program resources.
    handle_gui* gui_;
    handle_hri* hri_;

    // Bool flag.
    bool      success_              = false;

    // User input stuff.
    std::vector<std::string>  questionnaire_resp_ = {"Nada",
                                                     "Un poco",
                                                     "Moderadamente",
                                                     "Bastante",
                                                     "Muchísimo"};

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
                 {BT::InputPort<std::size_t>("_iteration")},
                 {BT::InputPort<double>("_questionnaire_score")},
                 {BT::OutputPort<double>("questionnaire_score_")}};
    }

    void init(handle_gui*       input_gui,
              handle_hri*       input_hri)
    {
      std::cout << "### Initializing UploadUserInput! ###" << std::endl;
      gui_      = input_gui;
      hri_      = input_hri;
    }

    // You must override the virtual function tick()
    BT::NodeStatus      tick() override;
    void                cleanup(bool halted);
    void                halt() override;

    // =================================================== 
    // Additional functionalities.

    // Check questionnaire input.
    void checkQuestionnaireInput(const std::size_t  &iteration,
                                       double       &questionnaire_score);

    // Inputs the Questionnaire answers.
    bool getQuestionnaireInput(const std::size_t  &iteration,
                                     double       &questionnaire_score);

    // TODO: Sent Answers to the database.
};
#endif // UploadUserInput_H_INCLUDED_