#ifndef CancelNavigationGoal_H_INCLUDED_
#define CancelNavigationGoal_H_INCLUDED_

// GENERAL INCLUDES
#include <iostream>
#include <math.h>

// ROS INCLUDES
#include <ros/ros.h>

// BT includes.
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// NHOA_BT INCLUDES
#include <plan_navigation.h>


/* TODO: This BT Action node encapsulates all the functionalities
         to cancel navigation goal.*/

class CancelNavigationGoal : public BT::CoroActionNode
{
  public:                                                                                                     

    // Shared program resources.
    plan_navigation* navigation_;

    // Bool flag.
    bool      success_              = false;

    // ==============

    CancelNavigationGoal(const std::string& name) : 
    BT::CoroActionNode(name, {})
    {
    }

    void init(plan_navigation*       input_navigation)
    {
      std::cout << "### Initializing CancelNavigationGoal! ###" << std::endl;

      navigation_ = input_navigation;
    }

    // You must override the virtual function tick()
    BT::NodeStatus      tick() override;
    void                cleanup(bool halted);
    void                halt() override;

    // =================================================== 
    // Additional functionalities.

    // Cancel Goal navigation goal.
    bool cancelGoal();

};
#endif // CancelNavigationGoal_H_INCLUDED_