#ifndef IsFirstUnfoldingPointFound_H_INCLUDED__
#define IsFirstUnfoldingPointFound_H_INCLUDED__

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

class IsFirstUnfoldingPointFound : public BT::ConditionNode
{
  public:
    IsFirstUnfoldingPointFound(const std::string& name) :
        BT::ConditionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
};
#endif //IsFirstUnfoldingPointFound_H_INCLUDED__
