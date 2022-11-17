#ifndef IsUserDetected_H_INCLUDED__
#define IsUserDetected_H_INCLUDED__

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

/* TODO: Here should be implemented all the functionalities of:
         - Body detection -> hri_fullbody (ROS4HRI) 
         - Face detection -> hri_facedetect (ROS4HRI) */

class IsUserDetected : public BT::ConditionNode
{
  public:
    IsUserDetected(const std::string& name) :
        BT::ConditionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
};
#endif //IsUserDetected_H_INCLUDED__
