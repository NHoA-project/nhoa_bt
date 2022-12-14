#ifndef IsApproximationReached_H_INCLUDED__
#define IsApproximationReached_H_INCLUDED__

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// NHOA_BT INCLUDES
#include <plan_navigation.h>

/* Approximation Navigation distance Condition Node */

class IsApproximationReached : public BT::ConditionNode
{
  public:

    // Shared program resources.
    plan_navigation* navigation_;

    // Boolean stuff.
    bool success_                 = false;

    // =================================================== 

    IsApproximationReached(const std::string& name) :
        BT::ConditionNode(name, {})
    {
    }

    void init(plan_navigation* input_navigation)
    {
      std::cout << "### Initializing IsApproximationReached! ###" << std::endl;
      navigation_    = input_navigation;
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    // =================================================== 
    // Check if user is engaging.
    bool isApproximationReached();
};
#endif //IsApproximationReached_H_INCLUDED__
