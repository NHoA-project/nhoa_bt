#include <CheckSmileScore.h>

// ROS INCLUDES
#include <ros/ros.h>

// NHOA_BT INCLUDES
#include <handle_hri.h>

// =====================================

BT::NodeStatus CheckSmileScore::tick()
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
        // =======
        if(!(checkSmileScore()))
        {
            std::cout << "ERROR! Not able to check smile score." << std::endl;
            success = false;
        }
        else
        {
            std::cout << "SUCCESS! The smile score is checked." << std::endl;
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


void CheckSmileScore::cleanup(bool halted)
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

void CheckSmileScore::halt(){

    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    // Do not forget to call this at the end.
    BT::CoroActionNode::halt();

}

// ####################
// Additional functions.

bool CheckSmileScore::checkSmileScore()
{
  std::cout << "### Checkin smile score -> " << hri_->set_smile_score() << "###" << std::endl;

  return true; 
}
