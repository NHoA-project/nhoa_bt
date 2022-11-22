#ifndef ExecuteNavigation_H_INCLUDED_
#define ExecuteNavigation_H_INCLUDED_

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

/* [TODO: Fill in this BT Action node class] This BT action node encapsulates the PAL ROBOTICS 
    ARI navigation functionality.*/

class ExecuteNavigation : public BT::CoroActionNode
{
  public:  

    // Shared program resources.
    plan_navigation* navigation_;

    // ROS stuff.
    ros::NodeHandle             nh_;    
        
    // MoveIt! stuff.
    bool success_                 = false;

    // =================================================== 

    ExecuteNavigation(const std::string& name, const BT::NodeConfiguration& config) : 
    BT::CoroActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        // This action has a single input port called "message"
        // Any port must have a name. The type is optional.
        return { BT::InputPort<std::vector<double>>("_navigation_goal") };
    }

    void init(ros::NodeHandle*  input_nh,
              plan_navigation*  input_navigation)
    {
      std::cout << "### Initializing ExecuteNavigation! ###" << std::endl;

      nh_           = *input_nh;
      navigation_   = input_navigation;
    }

    // You must override the virtual function tick()
    BT::NodeStatus      tick() override;
    void                cleanup(bool halted);
    void                halt() override;

    // =================================================== 

    // Execute Navigation by specifying a list of waypoints to the EE
    // to go through.
    bool executeNavigation(const std::vector<double>  &navigation_goal);
};
#endif // ExecuteNavigation_H_INCLUDED_