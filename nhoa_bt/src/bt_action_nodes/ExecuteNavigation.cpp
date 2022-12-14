#include <ExecuteNavigation.h>

// ROS INCLUDES
#include <ros/ros.h>

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
        auto navigation_goal  = getInput<std::vector<double>>("_navigation_goal");
        auto navigation_mode  = getInput<std::string>("_navigation_mode");
        auto rotation         = getInput<double>("_rotation");

        // =======
        if(navigation_mode.value().empty())
        {
          throw BT::RuntimeError("error reading port [navigation_mode]:", navigation_mode.error());
        }
        // --------
        else if(navigation_mode.value().compare("approach_navigation") == 0 )
        {
          if(!(executeApproachNavigation()))
          {
              std::cout << "ERROR! Not able to set the requested approach navigation." << std::endl;
              success = false;
          }
          else
          {
              std::cout << "SUCCESS! The requested approach navigation is reached." << std::endl;
              success = true;
          }
        }
        // --------
        else if(navigation_mode.value().compare("navigation_goal") == 0 )
        {
          if(!(executeNavigationGoal(navigation_goal.value())))
          {
              std::cout << "ERROR! Not able to set the requested navigation goal." << std::endl;
              success = false;
          }
          else
          {
              std::cout << "SUCCESS! The requested navigation_goal is reached." << std::endl;
              success = true;
          }
        }
        // --------
        else if(navigation_mode.value().compare("rotation") == 0 )
        {
          if(!(executeRotation(rotation.value())))
          {
              std::cout << "ERROR! Not able to set the requested rotation." << std::endl;
              success = false;
          }
          else
          {
              std::cout << "SUCCESS! The requested rotation is reached." << std::endl;
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


bool ExecuteNavigation::executeApproachNavigation()
{
  std::cout << "Executing approach navigation ..." << std::endl;

  return navigation_->set_approach_navigation();
}

bool ExecuteNavigation::executeNavigationGoal(const std::vector<double>  &navigation_goal)
{
  std::cout << "Executing navigation goal ..." << std::endl;

  return navigation_->set_navigation_goal(navigation_goal);
}

bool ExecuteNavigation::executeRotation(const double  &rotation)
{
  std::cout << "Executing rotation ..." << std::endl;

  return navigation_->set_rotation(rotation);
}