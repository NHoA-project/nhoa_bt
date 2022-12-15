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
        auto conversation_mode    = getInput<std::string>("_conversation_mode");
        auto iteration            = getInput<std::size_t>("_iteration");
        auto questionnaire_score  = getInput<double>("_questionnaire_score");

        // =======

        if((conversation_mode.value().empty()))
        {
          throw BT::RuntimeError("error reading port [conversation_mode]:", conversation_mode.error());
        }
        // --------
        else if(conversation_mode.value().compare("questionnaire") == 0 )
        {
          if(!(getQuestionnaireInput(iteration.value(), questionnaire_score.value())))
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
void  UploadUserInput::checkQuestionnaireInput(const std::size_t  &iteration,
                                                     double       &questionnaire_score)
{
  if( gui_->user_input_msg_.compare(questionnaire_resp_[0]) == 0 )
  {
    questionnaire_score += 1.0;
    // q_answers_ [iteration] = q_resp_[iteration][1];
  }
  else if( gui_->user_input_msg_.compare(questionnaire_resp_[1]) == 0 )  
  {
    questionnaire_score += 2.0;
    // q_answers_ [iteration] = q_resp_[iteration][2];
  }
  else if( gui_->user_input_msg_.compare(questionnaire_resp_[2]) == 0 )  
  {
    questionnaire_score += 3.0;
    // q_answers_ [iteration] = q_resp_[iteration][3];
  }
  else if( gui_->user_input_msg_.compare(questionnaire_resp_[3]) == 0 )  
  {
    questionnaire_score += 4.0;
    // q_answers_ [iteration] = q_resp_[iteration][4];
  }
  else if( gui_->user_input_msg_.compare(questionnaire_resp_[4]) == 0 )  
  {
    questionnaire_score += 5.0;
    // q_answers_ [iteration] = q_resp_[iteration][5];
  }
  
  if(iteration+1 == gui_->max_iter_)
  {
    // TODO: Integrate smile score.
    std::cout << " Adding smile score to the questionnaire one! " << std::endl;

    // questionnaire_score += hri_->set_smile_score();
  }
  std::cout << " ### Questionnaire score -> " << questionnaire_score << " ### " << std::endl;
  setOutput("questionnaire_score_", questionnaire_score);
}

bool UploadUserInput::getQuestionnaireInput(const std::size_t  &iteration,
                                                  double       &questionnaire_score)
{
  while (gui_->user_input_flag_ == false){}

  std::cout << "Introduce Question " << iteration << " answer: " << std::endl;
  std::cout << "Question " << iteration << " answer : '" << gui_->user_input_msg_ << "' sent to the DB cloud." << std::endl;

  UploadUserInput::checkQuestionnaireInput(iteration-1, questionnaire_score);

  gui_->user_input_flag_ = false;

  // TODO: Send answer to the DB (EUT).
  // uploadQuestionnaireAnswer(iteration, user_input);

  return true;
}

