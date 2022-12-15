#ifndef CheckSmileScore_H_INCLUDED_
#define CheckSmileScore_H_INCLUDED_

// GENERAL INCLUDES
#include <iostream>
#include <math.h>

// ROS INCLUDES
#include <ros/ros.h>

// BT includes.
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// NHOA_BT INCLUDES
#include <handle_hri.h>

/* This BT action node encapsulates the PAL ROBOTICS ARI "tts" 
   speech functionality.*/

class CheckSmileScore : public BT::CoroActionNode
{
  public:  

    // Shared program resources.
    handle_hri* hri_;
        
    // MoveIt! stuff.
    bool success_                 = false;

    // =================================================== 

    CheckSmileScore(const std::string& name) : 
    BT::CoroActionNode(name, {})
    {
    }

    void init(handle_hri*     input_hri)
    {
      std::cout << "### Initializing CheckSmileScore! ###" << std::endl;
      hri_    = input_hri;
    }

    // You must override the virtual function tick()
    BT::NodeStatus      tick() override;
    void                cleanup(bool halted);
    void                halt() override;

    // =================================================== 

    // Check smile score.
    bool checkSmileScore();
};
#endif // CheckSmileScore_H_INCLUDED_