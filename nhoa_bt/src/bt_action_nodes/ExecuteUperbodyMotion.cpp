#include <ExecuteUperbodyMotion.h>

// ROS INCLUDES
#include <ros/ros.h>

// NHOA_BT INCLUDES
#include <plan_motion.h>

// =====================================

BT::NodeStatus ExecuteUperbodyMotion::tick()
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
        auto motion_name = getInput<std::string>("_motion_name");

        // Debug 
        std::cout << "Motion name -> " << motion_name.value() << std::endl;

        // =======

        if((motion_name.value().empty()))
        {
          throw BT::RuntimeError("error reading port [motion_name]:", motion_name.error());
        }
        // --------
        else if((!motion_name.value().empty()))
        {
          if(!(executeUperbodyMotion(motion_name.value())))
          {
              std::cout << "ERROR! Not able to set the requested motion." << std::endl;
              success = false;
          }
          else
          {
              std::cout << "SUCCESS! The requested motion is reached." << std::endl;
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


void ExecuteUperbodyMotion::cleanup(bool halted)
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

void ExecuteUperbodyMotion::halt(){

    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    // Do not forget to call this at the end.
    BT::CoroActionNode::halt();

}

// ####################
// Additional functions.

bool ExecuteUperbodyMotion::executeUperbodyMotion(const std::string  &motion_name)
{
  std::cout << "### EXECUTING MOTION -> " << motion_name << " ###" << std::endl;

  return motion_->set_motion(motion_name);
}
