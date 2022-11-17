#include <DrawUserAttention.h>

// ROS INCLUDES
#include <ros/ros.h>

// RECYCLA INCLUDES
#include <plan_motion.h>

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
        auto iteration = getInput<std::size_t>("iteration");

        // =======

        // if((iteration.value().empty()))
        // {
        //   throw BT::RuntimeError("error reading port [iteration]:", iteration.error());
        // }
        // --------
        // else if((!iteration.value().empty()))
        // {
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
    std::cout << "### ITERATION -> " << iteration << " ###" << std::endl;
    DrawUserAttention::setOutputMotionName(iteration);
    DrawUserAttention::setOutputVoiceCmd(iteration);
    setOutput("iteration_", iteration + 1);
    succes_ = true;
  }
  else
  {
    succes_ = false;
  }

  return succes_;
}

void DrawUserAttention::setOutputMotionName(const std::size_t  &iteration)
{
  std::cout << "setOutputMotionName (output): " << motion_names[iteration] << std::endl;
  setOutput("motion_name_", motion_names[iteration]);
}

void DrawUserAttention::setOutputVoiceCmd(const std::size_t  &iteration)
{
  std::cout << "setOutputVoiceCmd (output): " << voice_cmds[iteration] << std::endl;
  setOutput("voice_cmd_", voice_cmds[iteration]);
}

