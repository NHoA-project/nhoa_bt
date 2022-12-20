#include <CancelNavigationGoal.h>

// ROS INCLUDES
#include <ros/ros.h>

// =====================================

BT::NodeStatus CancelNavigationGoal::tick()
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
        if(!(cancelGoal()))
        {
            std::cout << "ERROR! Not able to get cancel goal." << std::endl;
            success = false;
        }
        else
        {
            std::cout << "SUCCESS! cancel goal." << std::endl;
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


void CancelNavigationGoal::cleanup(bool halted)
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

void CancelNavigationGoal::halt(){

    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    // Do not forget to call this at the end.
    BT::CoroActionNode::halt();

}

// ####################
// Additional functions.
bool  CancelNavigationGoal::cancelGoal()
{
  std::cout << "Cancelling navigation goal ..." << std::endl;
  // return navigation_->cancel_goal();
  return true;
}