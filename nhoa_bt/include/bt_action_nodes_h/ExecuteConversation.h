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

// NHOA_BT INCLUDES
#include <handle_scene.h>

/* TODO: This BT Action node encapsulates all the functionalities
         for the User-Robot conversation.*/

class ExecuteConversation : public BT::CoroActionNode
{
  public:   

    // Shared program resources.
    handle_scene* scene_;

    // Predefined messages. TODO: Download voice commands from the DB (EUT).
    // - Agenda voice commands.
    // std::vector<std::string> agenda_cmds = {"Today is 12th of December and the weather is sunny.",
    //                                         "Remember you have to call your daughter and that you have an appointment with the doctor at 17:00."};       
    std::vector<std::string> agenda_cmds = {"Hoy es 12 de diciembre y el día es soleado.",
                                            "Recuerda que tienes que llamar a tu hija y que tienes una cita con el médico a las 17:00."};                                         

    // - Agenda recall voice commands.                                                 
    // std::vector<std::string> agenda_recall_cmds = {"We're done for now. Thank you.",
    //                                                 "Remember that today you must call your daughter and that you have an appointment with the doctor at 17:00. Bye bye!"};  
    std::vector<std::string> agenda_recall_cmds = {"Hemos terminado por ahora. Gracias.",
                                                    "Recuerda que tienes que llamar a tu hija y que tienes una cita con el médico a las 17:00. ¡Adiós!"};                                          

    // - Chit chat voice commands.                                        
    // std::vector<std::string> chit_chat_cmds = {"How are you today?",                                // Question
    //                                            "Did you sleep well?",                               // Y/N Question
    //                                            "Great. A good sleep is important.",                 // Yes
    //                                            "I am sorry to hear that. You must be very tired."}; // No
    std::vector<std::string> chit_chat_cmds = {"¿Cómo estás hoy?",                                // Question
                                               "¿Has dormido bien?"};                              // Y/N Question

    std::vector<std::string> chit_chat_fb_cmds = {"Genial. Dormir bien es importante.",              // Yes
                                                  "Lamento escuchar eso. Debes estar muy cansado."}; // No

    // -   Navigation feedback voice commands.
    std::vector<std::string> navigation_cmds     = {"¡Perdona, estás muy lejos! Podrías acercarte un poco porfavor."}; // Y/N Question


    // - Questionnaire voice commands.                                             
    // std::vector<std::string> questionnaire_cmds = {"Question 1: ...?",
    //                                                "Question 2: ...?",
    //                                                "Question 3: ...?"};   
    std::vector<std::string> questionnaire_cmds = {"¿Cuán lleno de energía te encuentras hoy?",
                                                   "¿Cuán activo te encuentras hoy?",
                                                   "¿Cuán animado te encuentras hoy?"};  

    // - Questionnaire feedback voice commands.
    // std::vector<std::string> questionnaire_fb_cmds = {"Today we finished it quicker",
    //                                                   "I see you're feeling better than yesterday.",
    //                                                   "I suggest calling a friend or family”, “Why don't you go out for a walk?"};      
    std::vector<std::string> questionnaire_fb_cmds = {"A veces solo necesitas desconectar. Pon tu canción favorita, cántala y veras que te sentirás mucho mejor. Préstale atención a la letra y siente la melodía.",
                                                      "Haz una lista de sitios donde te gustaría pasear y comienza por uno.",
                                                      "¿Qué tal si cocinas una receta saludable para toda tu familia o compañeros? Cocinar aporta grandes beneficios, como aumentar la creatividad y bienestar emocional.",
                                                      "¡Sal a caminar media hora hoy!"};        
    
    // - Questionnaire start voice commands.
    // std::vector<std::string> questionnaire_start_cmds = {"I would like to ask you a few questions. Is now a good moment for you?", // Y/N Question
    //                                                      "Great! Here I go.",             // Yes
    //                                                      "Let's do it some other time."}; // No
    std::vector<std::string> questionnaire_start_cmds     = {"Me gustaría hacerle algunas preguntas. ¿Es un buen momento para usted?"}; // Y/N Question
    std::vector<std::string> questionnaire_start_fb_cmds  = {"¡Genial! Allá voy.",             // Yes
                                                             "Lo dejamos para otro momento."}; // No   

    // - TODO: Segmentation scene voice commands.
    std::vector<std::string> segmentation_scene_cmds_     = {"",
                                                             "",
                                                             "",
                                                             "",
                                                             "",
                                                             "",
                                                             ""};                                                                                                                      

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
                 {BT::InputPort<double>("_questionnaire_score")}, 
                 {BT::InputPort<std::string>("_user_answer")}, 
                 {BT::OutputPort<std::size_t>("iteration_")},
                 {BT::OutputPort<std::string>("motion_name_")},
                 {BT::OutputPort<std::string>("voice_cmd_")} };
    }

    void init(handle_scene*  input_scene)
    {
      std::cout << "### Initializing ExecuteConversation! ###" << std::endl;

      scene_   = input_scene;
    }

    // You must override the virtual function tick()
    BT::NodeStatus      tick() override;
    void                cleanup(bool halted);
    void                halt() override;

    // =================================================== 
    // Additional functionalities.

    // Check scene.
    bool checkScene();

    // Outputs the Agenda "voice command".
    bool executeAgenda(const std::size_t  &iteration);

    // Outputs the Agenda Recall "motion_name" and "voice command".
    bool executeAgendaRecall(const std::size_t  &iteration);

    // Outputs Chit Chat the "voice command".
    bool executeChitChat(const std::size_t &iteration);

    // Outputs Chit Chat the "voice command".
    bool executeChitChatFB(const std::string &user_answer);

    // Outputs Navigation feedback "voice command".
    bool executeNavigationFB();

    // Outputs the Questionnaire "voice command".
    bool executeQuestionnaire(const std::size_t  &iteration);

    // Outputs the Questionnaire Feedback "voice command".
    bool executeQuestionnaireFB(const double  &questionnaire_score);

    // Outputs the "voice command" to start the Questionnaire conversation.
    bool startQuestionnaire(const std::size_t &iteration);

    // Outputs the "voice command" to start the Questionnaire FB conversation.
    bool startQuestionnaireFB(const std::string &user_answer);
};
#endif // ExecuteConversation_H_INCLUDED_