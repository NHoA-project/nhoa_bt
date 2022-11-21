#ifndef IsQuestionnaireRequested_H_INCLUDED__
#define IsQuestionnaireRequested_H_INCLUDED__

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

/* TODO: Dumb Condition node to Execute Questionnaire*/

class IsQuestionnaireRequested : public BT::ConditionNode
{
  public:
    IsQuestionnaireRequested(const std::string& name, const BT::NodeConfiguration& config) :
        BT::ConditionNode(name, config)
    {
    }
      
      
    static BT::PortsList providedPorts()
    {
        // This action has a single input port called "message"
        // Any port must have a name. The type is optional.
        return { BT::InputPort<std::string>("_questionnaire_requested") };
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
};
#endif //IsQuestionnaireRequested_H_INCLUDED__
