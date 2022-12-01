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

/* TODO: This BT Action node encapsulates all the functionalities
         for the User-Robot conversation.*/

class ExecuteConversation : public BT::CoroActionNode
{
  public:   

    // Predefined messages. TODO: Download voice commands from the DB (EUT).
    // std::vector<std::string> agenda_cmds = {"Today is 12th of December and the weather is sunny.",
    //                                         "Remember you have to call your daughter and that you have an appointment with the doctor at 17:00."};       
    std::vector<std::string> agenda_cmds = {"Hoy es 12 de diciembre y el día es soleado.",
                                            "Recuerda que tienes que llamar a tu hija y que tienes una cita con el médico a las 17:00."};                                         
                                                      
    // std::vector<std::string> agenda_recall_cmds = {"We're done for now. Thank you.",
    //                                                 "Remember that today you must call your daughter and that you have an appointment with the doctor at 17:00. Bye bye!"};  
    std::vector<std::string> agenda_recall_cmds = {"Hemos terminado por ahora. Gracias.",
                                                    "Recuerda que tienes que llamar a tu hija y que tienes una cita con el médico a las 17:00. ¡Adiós!"};                                          
                                              
    // std::vector<std::string> chit_chat_cmds = {"How are you today?",                                // Question
    //                                            "Did you sleep well?",                               // Y/N Question
    //                                            "Great. A good sleep is important.",                 // Yes
    //                                            "I am sorry to hear that. You must be very tired."}; // No
    std::vector<std::string> chit_chat_cmds = {"¿Cómo estás hoy?",                                // Question
                                               "¿Has dormido bien?",                              // Y/N Question
                                               "Genial. Dormir bien es importante.",              // Yes
                                               "Lamento escuchar eso. Debes estar muy cansado."}; // No
                                             
    // std::vector<std::string> questionnaire_cmds = {"Question 1: ...?",
    //                                                "Question 2: ...?",
    //                                                "Question 3: ...?"};   
    std::vector<std::string> questionnaire_cmds = {"Pregunta 1: ...?",
                                                   "Pregunta 2: ...?",
                                                   "Pregunta 3: ...?"};  

    // std::vector<std::string> questionnaire_fb_cmds = {"Today we finished it quicker",
    //                                                   "I see you're feeling better than yesterday.",
    //                                                   "I suggest calling a friend or family”, “Why don't you go out for a walk?"};      
    std::vector<std::string> questionnaire_fb_cmds = {"Hoy lo hemos terminado antes.",
                                                      "Veo que te sientes mejor que ayer.",
                                                      "Te sugiero que llames a un amigo o a un familiar. ¿Por qué no sales a dar un paseo?"};        
    
    // std::vector<std::string> questionnaire_start_cmds = {"I would like to ask you a few questions. Is now a good moment for you?", // Y/N Question
    //                                                      "Great! Here I go.",             // Yes
    //                                                      "Let's do it some other time."}; // No
    std::vector<std::string> questionnaire_start_cmds = {"Me gustaría hacerle algunas preguntas. ¿Es un buen momento para usted?", // Y/N Question
                                                         "¡Genial! Allá voy.",             // Yes
                                                         "Lo dejamos para otro momento."}; // No                                                                                                                             

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
                 {BT::OutputPort<std::string>("questionnaire_requested_")},
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
    // Outputs the Agenda "voice command".
    bool executeAgenda(const std::size_t  &iteration);

    // Outputs the Agenda Recall "motion_name" and "voice command".
    bool executeAgendaRecall(const std::size_t  &iteration);

    // Outputs Chit Chat the "voice command".
    bool executeChitChat(const std::size_t &iteration);

    // Outputs the Questionnaire "voice command".
    bool executeQuestionnaire(const std::size_t  &iteration);

    // Outputs the Questionnaire Feedback "voice command".
    bool executeQuestionnaireFB(const std::size_t  &iteration);

    // Outputs the "voice command" to start the Questionnaire conversation.
    bool startQuestionnaire(const std::size_t &iteration);
};
#endif // ExecuteConversation_H_INCLUDED_