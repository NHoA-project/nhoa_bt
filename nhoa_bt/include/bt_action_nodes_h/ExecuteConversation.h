#ifndef ExecuteConversation_H_INCLUDED_
#define ExecuteConversation_H_INCLUDED_

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


class ExecuteConversation : public BT::CoroActionNode
{
  public:   

    // TODO: Predefined messages.
    std::vector<std::string> chit_chat_cmds = {"How are you?",
                                               "Did you sleep well?",
                                               "Today is 12th of December and the weather is sunny.",
                                               "Remember you have to call your daughter and that you have an appointment with the doctor at 17:00."};                                         

    std::vector<std::string> questionnaire_cmds = {"I would like to ask you a few questions. Is now a good moment for you?",
                                                   "Question 1: ...?",
                                                   "Question 2: ...?",
                                                   "Question 3: ...?"};   

    std::vector<std::string> questionnaire_fb_cmds = {"Today we finished it quicker",
                                                      "I see you're feeling better than yesterday.",
                                                      "I suggest calling a friend or family”, “Why don't you go out for a walk?"};            
                                                      
    std::vector<std::string> agenda_recall_cmds = {"We're done for now. Thank you.",
                                                    "Remember that today you must call your daughter and that you have an appointment with the doctor at 17:00. Bye bye!"};                                          
                                                                                         


    // Bool flag.
    bool success_ = false;


    // ==============

    ExecuteConversation(const std::string& name, const BT::NodeConfiguration& config) : 
    BT::CoroActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        // This action has a single input port called "message"
        // Any port must have a name. The type is optional.
        return { {BT::InputPort<std::string>("_conversation_mode")},
                 {BT::InputPort<std::size_t>("_iteration")}, 
                 {BT::OutputPort<std::size_t>("iteration_")},
                 {BT::OutputPort<std::string>("motion_name_")},
                 {BT::OutputPort<std::string>("voice_cmd_")} };
    }

    void init()
    {
      std::cout << "### Initializing ExecuteConversation! ###" << std::endl;
    }

    // You must override the virtual function tick()
    BT::NodeStatus      tick() override;
    void                cleanup(bool halted);
    void                halt() override;

    // =================================================== 
    // Additional functionalities.

    // Outputs the Agenda Recall "motion_name" and "voice command".
    bool executeAgendaRecall(const std::size_t  &iteration);

    // Outputs the Chit Chat "voice command".
    bool executeChitChat(const std::size_t  &iteration);

    // Outputs the Questionnaire "voice command".
    bool executeQuestionnaire(const std::size_t  &iteration);

    // Outputs the Questionnaire Feedback "voice command".
    bool executeQuestionnaireFB(const std::size_t  &iteration);
};
#endif // ExecuteConversation_H_INCLUDED_