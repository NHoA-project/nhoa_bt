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

    
    // EUT DIGITAL stuff.
    std::vector<std::string> q_answers_ = {"", "", ""};

    std::vector<std::vector<std::string>>  q_resp_ = {{"2f83e34b-b288-4108-89f8-a8dd286c8339",
                                                       "6a029077-5c26-47ce-8ad4-03b3e32c678c",
                                                       "02bd3ffd-8a0e-4a81-97d3-8c400a54c1c7",
                                                       "072fddf1-0f72-4789-b491-bc995dc3b844",
                                                       "dde68135-e9ea-462a-a037-bd8f1e1ebdde"},
                                                      {"2a56b59d-7869-4845-a8c3-0f9c18fb8338",
                                                       "22f5a6ba-309d-4ff0-a979-44d6946a30b7",
                                                       "b2379d4b-263f-4196-b106-0ca7bb9a10bf",
                                                       "9c44e6db-3ff7-4fed-aad1-d34106bd15ba",
                                                       "f65fb160-4e9f-41c6-8daa-d2efa2762f1a"},
                                                      {"22f5a6ba-309d-4ff0-a979-44d6946a30b7",
                                                       "8852993f-4507-49ef-a1c0-696c5ce8b88a",
                                                       "a6e1e340-d6ef-43f4-838e-929f542aa1e2",
                                                       "85e3689e-6fb6-414a-ba88-8d677efa5b20",
                                                       "d1e57313-b766-48fc-ac48-eb5b302fb3cc"}};
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