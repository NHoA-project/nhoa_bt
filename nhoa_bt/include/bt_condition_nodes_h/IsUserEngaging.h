#ifndef IsUserEngaging_H_INCLUDED__
#define IsUserEngaging_H_INCLUDED__

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

/* TODO: Here should be implemented all the functionalities of:
         - Body detection -> pyhri (ROS4HRI) */

class IsUserEngaging : public BT::ConditionNode
{
  public:
    IsUserEngaging(const std::string& name) :
        BT::ConditionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
};
#endif //IsUserEngaging_H_INCLUDED__
