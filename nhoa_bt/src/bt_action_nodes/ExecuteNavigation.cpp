#include <ExecuteNavigation.h>

// ROS INCLUDES
#include <ros/ros.h>

// NHOA_BT INCLUDES
#include <plan_navigation.h>

// =====================================

BT::NodeStatus ExecuteNavigation::tick()
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
        auto navigation_goal = getInput<std::vector<double>>("_navigation_goal");

        // =======
        if(!(executeNavigation(navigation_goal.value())))
        {
            std::cout << "ERROR! Not able to set the requested navigation goal." << std::endl;
            success = false;
        }
        else
        {
            std::cout << "SUCCESS! The requested navigation_goal is reached." << std::endl;
            success = true;
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


void ExecuteNavigation::cleanup(bool halted)
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

void ExecuteNavigation::halt(){

    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    // Do not forget to call this at the end.
    BT::CoroActionNode::halt();

}

// ####################
// Additional functions.

bool ExecuteNavigation::executeNavigation(const std::vector<double>  &navigation_goal)
{
  std::cout << "Executin navigation goal ..." << std::endl;

  return navigation_->set_navigation_goal(navigation_goal);
}
