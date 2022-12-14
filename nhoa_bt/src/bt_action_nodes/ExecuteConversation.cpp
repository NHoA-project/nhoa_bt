#include <ExecuteConversation.h>

// ROS INCLUDES
#include <ros/ros.h>

// STD INCLUDES
#include <cstdlib>

// =====================================

BT::NodeStatus ExecuteConversation::tick()
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
        auto user_answer          = getInput<std::string>("_user_answer");

        // =======
        if((conversation_mode.value().empty()))
        {
          throw BT::RuntimeError("error reading port [conversation_mode]:", conversation_mode.error());
        }
        // --------
        else if((conversation_mode.value().compare("agenda") == 0) )
        {
          if(!(executeAgenda(iteration.value())))
          {
            std::cout << "ERROR! Not able to execute Agenda conversation." << std::endl;
            success = false;
          }
          else
          {
            std::cout << "SUCCESS! Agenda conversation executed." << std::endl;
            success = true;
          }
        }
        // --------
        else if((conversation_mode.value().compare("agenda_recall") == 0) )
        {
          if(!(executeAgendaRecall(iteration.value())))
          {
              std::cout << "ERROR! Not able to execute Agenda Recall." << std::endl;
              success = false;
          }
          else
          {
            std::cout << "SUCCESS! Agenda Recall executed." << std::endl;
            success = true;
          }
        }
        // --------
        else if(conversation_mode.value().compare("chit_chat") == 0)
        {
          if(!(executeChitChat(iteration.value())))
          {
              std::cout << "ERROR! Not able to start Chit Chat." << std::endl;
              success = false;
          }
          else
          {
              std::cout << "SUCCESS! Starting Chit Chat conversation is fullfilled." << std::endl;
              success = true;
          }
        }
        // --------
        else if(conversation_mode.value().compare("chit_chat_fb") == 0)
        {
          if(!(executeChitChatFB(user_answer.value())))
          {
              std::cout << "ERROR! Not able to start Chit Chat FB." << std::endl;
              success = false;
          }
          else
          {
              std::cout << "SUCCESS! Starting Chit Chat FB conversation is fullfilled." << std::endl;
              success = true;
          }
        }
        // --------
        else if((conversation_mode.value().compare("questionnaire") == 0 ) )
        {
          if(!(executeQuestionnaire(iteration.value())))
          {
            std::cout << "ERROR! Not able to execute Questionnaire." << std::endl;
            success = false;
          }
          else
          {
            std::cout << "SUCCESS! Questionnaire executed." << std::endl;
            success = true;
          }
        }
        // --------
        else if((conversation_mode.value().compare("questionnaire_fb") == 0) )
        {
          if(!(executeQuestionnaireFB(questionnaire_score.value())))
          {
              std::cout << "ERROR! Not able to execute Questionnaire Feedback." << std::endl;
              success = false;
          }
          else
          {
            std::cout << "SUCCESS! Questionnaire Feedback executed." << std::endl;
            success = true;
          }
        }
        // --------
        else if(conversation_mode.value().compare("questionnaire_start") == 0)
        {
          if(!(startQuestionnaire(iteration.value())))
          {
              std::cout << "ERROR! Not able to start Questionnaire." << std::endl;
              success = false;
          }
          else
          {
              std::cout << "SUCCESS! Starting Questionnaire is fullfilled." << std::endl;
              success = true;
          }
        }
        // --------
        else if(conversation_mode.value().compare("questionnaire_start_fb") == 0)
        {
          if(!(startQuestionnaireFB(user_answer.value())))
          {
              std::cout << "ERROR! Not able to start Questionnaire FB." << std::endl;
              success = false;
          }
          else
          {
              std::cout << "SUCCESS! Starting Questionnaire FB is fullfilled." << std::endl;
              success = true;
          }
        }
        // --------
        else if(conversation_mode.value().compare("scene_segmentation") == 0)
        {
          if(!(checkScene()))
          {
              std::cout << "ERROR! Not able to Check Scene." << std::endl;
              success = false;
          }
          else
          {
              std::cout << "SUCCESS! Check Scene is fullfilled." << std::endl;
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


void ExecuteConversation::cleanup(bool halted)
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

void ExecuteConversation::halt(){

    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    // Do not forget to call this at the end.
    BT::CoroActionNode::halt();

}

// ####################
// Additional functions.

bool ExecuteConversation::checkScene()
{
  // TODO: Depending on the Boolean variables output some voice command.
  std::cout << "### person_sitting_on_chair -> " << scene_->scene_digest_msg_.person_sitting_on_chair << " ###" << std::endl;
  std::cout << "### person_holding_bottle -> " << scene_->scene_digest_msg_.person_holding_bottle << " ###" << std::endl;
  std::cout << "### person_holding_cup -> " << scene_->scene_digest_msg_.person_holding_cup << " ###" << std::endl;
  std::cout << "### person_looking_at_tv -> " << scene_->scene_digest_msg_.person_looking_at_tv << " ###" << std::endl;
  std::cout << "### cup_on_table -> " << scene_->scene_digest_msg_.cup_on_table << " ###" << std::endl;
  std::cout << "### bottle_on_table -> " << scene_->scene_digest_msg_.bottle_on_table << " ###" << std::endl;
  std::cout << "### chair_near_table -> " << scene_->scene_digest_msg_.chair_near_table << " ###" << std::endl;

  // Higher probability sorted.
  if(scene_->scene_digest_msg_.person_sitting_on_chair)
  {
    setOutput("voice_cmd_", segmentation_scene_cmds_[0]);
    success_ = true;
  }
  // -----
  else if(scene_->scene_digest_msg_.person_holding_bottle)
  {
    setOutput("voice_cmd_", segmentation_scene_cmds_[1]);
    success_ = true;
  }
  // -----
  else if(scene_->scene_digest_msg_.person_holding_cup)
  {
    setOutput("voice_cmd_", segmentation_scene_cmds_[2]);
    success_ = true;
  }
  // -----
  else if(scene_->scene_digest_msg_.cup_on_table)
  {
    setOutput("voice_cmd_", segmentation_scene_cmds_[3]);
    success_ = true;
  }
  // -----
  else if(scene_->scene_digest_msg_.bottle_on_table)
  {
    setOutput("voice_cmd_", segmentation_scene_cmds_[4]);
    success_ = true;
  }
  // -----
  else if(scene_->scene_digest_msg_.chair_near_table)
  {
    setOutput("voice_cmd_", segmentation_scene_cmds_[5]);
    success_ = true;
  }
  // -----
  else if(scene_->scene_digest_msg_.person_looking_at_tv)
  {
    setOutput("voice_cmd_", segmentation_scene_cmds_[6]);
  }
  // ----
  else
  {
    success_ = false;
  }
  return success_;
}

bool ExecuteConversation::executeAgenda(const std::size_t  &iteration)
{
  if(iteration < agenda_cmds.size())
  {
    setOutput("iteration_", iteration + 1);  
    setOutput("voice_cmd_", agenda_cmds[iteration]);
    success_ = true;
  }
  else
  {
    success_ = false;
  }
  return success_;
}

bool ExecuteConversation::executeAgendaRecall(const std::size_t  &iteration)
{
  if(iteration < agenda_recall_cmds.size())
  {
    setOutput("iteration_", iteration + 1);  
    setOutput("voice_cmd_", agenda_recall_cmds[iteration]);
    if( iteration == agenda_recall_cmds.size()-1 )
    {
      setOutput("motion_name_", "wave");
    }
    success_ = true;
  }
  else
  {
    success_ = false;
  }
  return success_;
}

bool ExecuteConversation::executeChitChat(const std::size_t &iteration)
{
  if(iteration < chit_chat_cmds.size())
  {
    setOutput("iteration_", iteration + 1);  
    setOutput("voice_cmd_", chit_chat_cmds[iteration]);
    success_ = true;
  }
  else
  {
    success_ = false;
  }
  return success_;
}

bool ExecuteConversation::executeChitChatFB(const std::string &user_answer)
{
  if( user_answer.compare("true") == 0 )
  {
    setOutput("voice_cmd_", chit_chat_fb_cmds[0]);
    success_ = true;
  }
  else if( user_answer.compare("false") == 0 )
  { 
    setOutput("voice_cmd_", chit_chat_fb_cmds[1]);
    success_ = true;
  }
  else
  {
    success_ = false;
  }
  return success_;
}

bool ExecuteConversation::executeQuestionnaire(const std::size_t  &iteration)
{
  if(iteration < questionnaire_cmds.size())
  {
    setOutput("iteration_", iteration + 1);  
    setOutput("voice_cmd_", questionnaire_cmds[iteration]);
    success_ = true;
  }
  else
  {
    success_ = false;
  }
  return success_;
}


bool ExecuteConversation::executeQuestionnaireFB(const double  &questionnaire_score)
{

  if(questionnaire_score < 5.0)
  {
    setOutput("voice_cmd_", questionnaire_fb_cmds[0]);
  }
  else if(questionnaire_score < 10.0)
  {
    setOutput("voice_cmd_", questionnaire_fb_cmds[1]);
  }
  else if(questionnaire_score < 15.0)
  {
    setOutput("voice_cmd_", questionnaire_fb_cmds[2]);
  }
  else
  {
    setOutput("voice_cmd_", questionnaire_fb_cmds[3]);
  }
  success_ = true;
  return success_;
}

bool ExecuteConversation::startQuestionnaire(const std::size_t &iteration)
{
  if (iteration < questionnaire_start_cmds.size() )
  {
    setOutput("iteration_", iteration + 1);  
    setOutput("voice_cmd_", questionnaire_start_cmds[iteration]);
    success_ = true;
  }
  else
  { 
    success_ = false;
  }
  return success_;
}

bool ExecuteConversation::startQuestionnaireFB(const std::string &user_answer)
{
  if( user_answer.compare("true") == 0 )
  {
    setOutput("voice_cmd_", questionnaire_start_fb_cmds[0]);
    success_ = true;
  }
  else if( user_answer.compare("false") == 0 )
  { 
    setOutput("voice_cmd_", questionnaire_start_fb_cmds[1]);
    success_ = true;
  }
  else
  {
    success_ = false;
  }
  return success_;
}
