#include <ExecuteHeadMotion.h>

// ROS INCLUDES
#include <ros/ros.h>

// NHOA_BT INCLUDES
#include <plan_motion.h>

// =====================================

BT::NodeStatus ExecuteHeadMotion::tick()
{   
  std::string input;
  bool waiting = true;
  bool success = false;
  while (waiting) 
  {
      std::cout << name() << ": Select 's' (start) or 'p'(pause)" << std::endl;
      std::cin >> input;
      if (input.compare("s") == 0) 
      { 
        // Get the Blackboard input arguments.
        auto head_motion_mode = getInput<std::string>("_head_motion_mode");
        auto follow_traj_goal = getInput<std::vector<double>>("_pan_tilt_goal");
        auto point_head       = getInput<geometry_msgs::PointStamped>("_point_head_goal");

        // Debug 
        std::cout << "Head motion mode -> " << head_motion_mode.value() << std::endl;

        // =======

        if((head_motion_mode.value().empty()))
        {
          throw BT::RuntimeError("error reading port [head_motion_mode]:", head_motion_mode.error());
        }
        // --------
        else if( head_motion_mode.value().compare("point_head") == 0)
        {
          if(!(executePointHead(point_head.value())))
          {
            std::cout << "ERROR! Not able to set the requested Point Head." << std::endl;
            success = false;
          }
          else
          {
            std::cout << "SUCCESS! The Point Head is reached." << std::endl;
            success = true;
          }
        }
        // --------
        else if( head_motion_mode.value().compare("pan_tilt") == 0)
        {
          if(!(executeFollowJointTraj(follow_traj_goal.value())))
          {
            std::cout << "ERROR! Not able to set the requested Pan Tilt." << std::endl;
            success = false;
          }
          else
          {
            std::cout << "SUCCESS! The Pan Tilt is reached." << std::endl;
            success = true;
          }
        }
        waiting = false;
      } 
  }

  std::cout << "Asyncronous process finished!"<<std::endl;
  cleanup(false);
  if(success)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}


void ExecuteHeadMotion::cleanup(bool halted)
{
    std::cout << "cleaning up" << std::endl;
    if(halted)
    {
        std::cout << name() <<": cleaning up after an halt()\n" << std::endl;
    }
    else{
        std::cout << name() <<": cleaning up after SUCCESS\n" << std::endl;
    }
}

void ExecuteHeadMotion::halt(){

    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    // Do not forget to call this at the end.
    BT::CoroActionNode::halt();

}

// ####################
// Additional functions

bool ExecuteHeadMotion::executeFollowJointTraj(const std::vector<double>  &joint)
{
  std::cout << "### EXECUTING FOLLOW JOINT TRAJ ###" << std::endl;

  return head_motion_->set_joint_trajectory_goal(joint);
}

bool ExecuteHeadMotion::executePointHead(const geometry_msgs::PointStamped   &point)
{
  std::cout << "### EXECUTING POINT HEAD ###" << std::endl;

  return head_motion_->set_point_head_goal(point);
}
