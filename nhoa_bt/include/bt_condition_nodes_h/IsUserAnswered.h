#ifndef IsUserAnswered_H_INCLUDED__
#define IsUserAnswered_H_INCLUDED__

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// NHOA_BT INCLUDES
#include <handle_hri.h>

/* TODO: Dumb Condition node to Execute Questionnaire*/

class IsUserAnswered : public BT::ConditionNode
{
  public:

    // Shared program resources.
    handle_hri* hri_;  
    
    // Boolean stuff.
    bool success_                 = false;

    // =================================================== 


    IsUserAnswered(const std::string& name, const BT::NodeConfiguration& config) :
        BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        // This condition has a single input port called "message"
        // Any port must have a name. The type is optional.
        return { {BT::InputPort<std::string>("_conversation_mode")},
                 {BT::OutputPort<std::string>("user_answered_")} };
    }
    
    void init(handle_hri* input_hri)
    {
      std::cout << "### Initializing IsUserAnswered! ###" << std::endl;
      hri_    = input_hri;
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    // =================================================== 
    // Check if user has answered.
    bool isUserAnswered(const std::string &conversation_mode);
};
#endif //IsUserAnswered_H_INCLUDED__
