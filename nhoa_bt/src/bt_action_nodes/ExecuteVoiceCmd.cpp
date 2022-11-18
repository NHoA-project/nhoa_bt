#include <ExecuteVoiceCmd.h>

// ROS INCLUDES
#include <ros/ros.h>

// =====================================

BT::NodeStatus ExecuteVoiceCmd::tick()
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
        auto voice_cmd = getInput<std::string>("_voice_cmd");

        // =======

        if((voice_cmd.value().empty()))
        {
          throw BT::RuntimeError("error reading port [voice_cmd]:", voice_cmd.error());
        }
        // --------
        else if((!voice_cmd.value().empty()))
        {
          if(!(executeVoiceCmd(voice_cmd.value())))
          {
              std::cout << "ERROR! Not able to execute voice cmd." << std::endl;
              success = false;
          }
          else
          {
              std::cout << "SUCCESS! Voice cmd executed." << std::endl;
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


void ExecuteVoiceCmd::cleanup(bool halted)
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

void ExecuteVoiceCmd::halt(){

    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    // Do not forget to call this at the end.
    BT::CoroActionNode::halt();

}

// ####################
// Additional functions.

bool ExecuteVoiceCmd::executeVoiceCmd(const std::string  &voice_cmd)
{
  std::cout << "Robot says -> " << voice_cmd << std::endl;

  return true;
}

