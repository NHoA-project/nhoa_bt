#include <UploadUserInput.h>

// ROS INCLUDES
#include <ros/ros.h>

// =====================================

BT::NodeStatus UploadUserInput::tick()
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
        auto conversation_mode  = getInput<std::string>("_conversation_mode");
        auto iteration          = getInput<std::size_t>("_iteration");

        // =======

        if((conversation_mode.value().empty()))
        {
          throw BT::RuntimeError("error reading port [conversation_mode]:", conversation_mode.error());
        }
        // --------
        else if(conversation_mode.value().compare("questionnaire") == 0 )
        {
          if(!(getQuestionnaireInput(iteration.value())))
          {
              std::cout << "ERROR! Not able to get Questionnaire Input." << std::endl;
              success = false;
          }
          else
          {
              std::cout << "SUCCESS! Getting Questionnaire Input." << std::endl;
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


void UploadUserInput::cleanup(bool halted)
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

void UploadUserInput::halt(){

    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    // Do not forget to call this at the end.
    BT::CoroActionNode::halt();

}

// ####################
// Additional functions.

bool UploadUserInput::getQuestionnaireInput(const std::size_t  &iteration)
{
  waiting_user = true;

  while (waiting_user)
  {
    std::cout << "Introduce Question " << iteration << " answer: " << std::endl;
    std::cin  >> user_input;
    std::cout << "Question " << iteration << " answer : '" << user_input << "' sent to the DB cloud." << std::endl;

    // TODO: Send answer to the DB (EUT).
    // uploadQuestionnaireAnswer(iteration, user_input);
    waiting_user = false;
  }

  return true;
}

