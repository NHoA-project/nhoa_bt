#include <DrawUserAttention.h>

// ROS INCLUDES
#include <ros/ros.h>

// =====================================

BT::NodeStatus DrawUserAttention::tick()
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
        auto iteration = getInput<std::size_t>("_iteration");

        // =======
        if(!(drawUserAttention(iteration.value())))
        {
            std::cout << "ERROR! Not able to draw user attention action." << std::endl;
            success = false;
        }
        else
        {
            std::cout << "SUCCESS! Drawing user attention action is reached." << std::endl;
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


void DrawUserAttention::cleanup(bool halted)
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

void DrawUserAttention::halt(){

    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    // Do not forget to call this at the end.
    BT::CoroActionNode::halt();

}

// ####################
// Additional functions.

bool DrawUserAttention::drawUserAttention(const std::size_t  &iteration)
{
  if(!(iteration > voice_cmds.size()))
  {
    std::cout << "### Executing -> " << motion_names_[iteration] << " ###" << std::endl;

    // Setting outputs.
    setOutput("iteration_", iteration + 1);  
    setOutput("motion_name_", motion_names_[iteration]);
    setOutput("voice_cmd_", voice_cmds[iteration]);

    success_ = true;
  }
  else
  {
    success_ = false;
  }

  return success_;
}


