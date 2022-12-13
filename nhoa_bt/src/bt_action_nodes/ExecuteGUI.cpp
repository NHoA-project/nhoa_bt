#include <ExecuteGUI.h>

// ROS INCLUDES
#include <ros/ros.h>

// NHOA_BT INCLUDES
#include <handle_gui.h>

// =====================================

BT::NodeStatus ExecuteGUI::tick()
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
        auto iteration  = getInput<std::size_t>("_iteration");
        auto web_type   = getInput<int>("_web_type");

        // =======

        // if((web_name.value().empty()))
        // {
        //   throw BT::RuntimeError("error reading port [web_name]:", web_name.error());
        // }
        // // --------
        // else if((!web_name.value().empty()))
        // {
          if(!(executeGUI(iteration.value(), web_type.value())))
          {
              std::cout << "ERROR! Not able to set the requested web name." << std::endl;
              success = false;
          }
          else
          {
              std::cout << "SUCCESS! The requested web name is reached." << std::endl;
              success = true;
          }
        // }
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


void ExecuteGUI::cleanup(bool halted)
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

void ExecuteGUI::halt(){

    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    // Do not forget to call this at the end.
    BT::CoroActionNode::halt();

}

// ####################
// Additional functions.

bool ExecuteGUI::executeGUI(const std::size_t  &iteration,
                            const int          &web_type)
{
  // std::cout << "### ITERATION ->  "<< iteration << "###" << std::endl;
  return gui_->set_web_go_to(iteration, web_type); 
}
