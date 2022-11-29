#ifndef ExecuteHeadMotion_H_INCLUDED_
#define ExecuteHeadMotion_H_INCLUDED_

// GENERAL INCLUDES
#include <iostream>
#include <math.h>

// ROS INCLUDES
#include <ros/ros.h>

// BT includes.
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// NHOA_BT INCLUDES
#include <plan_head_motion.h>

/* This BT action node encapsulates the PAL ROBOTICS ARI head 
   motion functionality.*/

class ExecuteHeadMotion : public BT::CoroActionNode
{
  public:  

    // Shared program resources.
    plan_head_motion* head_motion_;

    // ROS stuff.
    ros::NodeHandle             nh_;    
        
    // MoveIt! stuff.
    bool success_                 = false;

    // =================================================== 

    ExecuteHeadMotion(const std::string& name, const BT::NodeConfiguration& config) : 
    BT::CoroActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        // This action has a single input port called "message"
        // Any port must have a name. The type is optional.
        return { {BT::InputPort<std::string>("_head_motion_mode")},
                 {BT::InputPort<std::vector<double>>("_pan_tilt_value")},
                 {BT::InputPort<geometry_msgs::PointStamped>("_point_target_value")}
               };
                //  {BT::OutputPort<std::string>("move_group_")}};
    }

    void init(ros::NodeHandle*  input_nh,
              plan_head_motion* input_head_motion)
    {
      std::cout << "### Initializing ExecuteHeadMotion! ###" << std::endl;

      nh_             = *input_nh;
      head_motion_    = input_head_motion;
    }

    // You must override the virtual function tick()
    BT::NodeStatus      tick() override;
    void                cleanup(bool halted);
    void                halt() override;

    // =================================================== 

    bool executeFollowJointTraj(const std::vector<double>     &joint);

    bool executePointHead(const geometry_msgs::PointStamped   &point);
};
#endif // ExecuteHeadMotion_H_INCLUDED_